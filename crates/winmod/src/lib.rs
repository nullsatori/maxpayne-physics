//! Shared plumbing for DLLs injected by an ASI loader. Windows only.
#![cfg(windows)]

use std::ffi::{OsString, c_void};
use std::fmt;
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::os::windows::ffi::OsStringExt;
use std::path::{Path, PathBuf};
use std::ptr::{null, null_mut};
use std::sync::OnceLock;
use std::time::Instant;

use windows_sys::Win32::Foundation::{CloseHandle, MAX_PATH};
use windows_sys::Win32::System::LibraryLoader::{DisableThreadLibraryCalls, GetModuleFileNameW};
use windows_sys::Win32::System::ProcessStatus::EnumProcessModules;
use windows_sys::Win32::System::SystemServices::DLL_PROCESS_ATTACH;
use windows_sys::Win32::System::Threading::{CreateThread, GetCurrentProcess, GetCurrentProcessId};

use windows_sys::Win32::Foundation::HMODULE;

/// A loaded module handle (its base address).
#[derive(Clone, Copy, Debug)]
pub struct Module(HMODULE);

impl Module {
    /// The process executable.
    pub const PROCESS: Module = Module(null_mut());

    pub fn base(self) -> usize {
        self.0 as usize
    }
}

#[doc(hidden)]
pub use windows_sys::Win32::Foundation::HMODULE as RawModule;

/// Declares the DLL entry point; `init(module)` then runs on its own thread,
/// outside the loader lock.
#[macro_export]
macro_rules! asi_entry {
    ($init:path) => {
        #[unsafe(no_mangle)]
        extern "system" fn DllMain(module: $crate::RawModule, reason: u32, _reserved: *mut ::core::ffi::c_void) -> i32 {
            // SAFETY: called by the OS loader with this DLL's own handle.
            unsafe { $crate::on_attach(module, reason, $init) }
        }
    };
}

struct Start {
    module: Module,
    init: fn(Module),
}

/// # Safety
/// Must only be called from `DllMain` with the handle the loader passed in.
#[doc(hidden)]
pub unsafe fn on_attach(module: HMODULE, reason: u32, init: fn(Module)) -> i32 {
    if reason != DLL_PROCESS_ATTACH {
        return 1;
    }
    let start = Box::into_raw(Box::new(Start {
        module: Module(module),
        init,
    }));
    // SAFETY: Win32 calls with valid arguments; `start` is freed by the
    // thread, or here if the thread was not created.
    unsafe {
        DisableThreadLibraryCalls(module);
        let thread = CreateThread(null(), 0, Some(thread_main), start as *const c_void, 0, null_mut());
        if thread.is_null() {
            drop(Box::from_raw(start));
            return 0;
        }
        CloseHandle(thread);
    }
    1
}

static PANIC_LOG: OnceLock<PathBuf> = OnceLock::new();

unsafe extern "system" fn thread_main(param: *mut c_void) -> u32 {
    // SAFETY: `param` is the Box leaked in on_attach, consumed exactly once.
    let start = unsafe { Box::from_raw(param as *mut Start) };
    let log_path = log_path(start.module);
    let _ = PANIC_LOG.set(log_path);
    std::panic::set_hook(Box::new(|info| {
        if let Some(path) = PANIC_LOG.get()
            && let Ok(mut file) = OpenOptions::new().append(true).create(true).open(path)
        {
            let _ = writeln!(file, "PANIC: {info}");
        }
    }));
    // A panic must not unwind into the game; the hook above records it.
    match std::panic::catch_unwind(|| (start.init)(start.module)) {
        Ok(()) => 0,
        Err(_) => 1,
    }
}

/// `<dll name>.log` next to the injected DLL.
pub fn log_path(module: Module) -> PathBuf {
    module_path(module).with_extension("log")
}

/// Line log with seconds since creation; flushes every line so it survives crashes.
pub struct Log {
    file: Option<File>,
    start: Instant,
}

impl Log {
    pub fn create(path: &Path) -> Self {
        Self {
            file: File::create(path).ok(),
            start: Instant::now(),
        }
    }

    pub fn line(&mut self, args: fmt::Arguments) {
        if let Some(file) = &mut self.file {
            let _ = writeln!(file, "[{:9.3}] {args}", self.start.elapsed().as_secs_f64());
            let _ = file.flush();
        }
    }
}

#[macro_export]
macro_rules! log {
    ($log:expr, $($arg:tt)*) => { $log.line(format_args!($($arg)*)) };
}

/// Full path of a loaded module.
pub fn module_path(module: Module) -> PathBuf {
    let mut buf = [0u16; MAX_PATH as usize];
    // SAFETY: buffer pointer and length match; Module holds a loader handle.
    let len = unsafe { GetModuleFileNameW(module.0, buf.as_mut_ptr(), buf.len() as u32) } as usize;
    PathBuf::from(OsString::from_wide(&buf[..len]))
}

/// Base address and path of every module in the process.
pub fn loaded_modules() -> Vec<(Module, PathBuf)> {
    let mut modules: Vec<HMODULE> = vec![null_mut(); 1024];
    let mut needed = 0u32;
    // SAFETY: buffer pointer and byte size match.
    let ok = unsafe {
        EnumProcessModules(
            GetCurrentProcess(),
            modules.as_mut_ptr(),
            (modules.len() * size_of::<HMODULE>()) as u32,
            &mut needed,
        )
    };
    if ok == 0 {
        return Vec::new();
    }
    modules.truncate((needed as usize / size_of::<HMODULE>()).min(modules.len()));
    modules
        .into_iter()
        .map(|m| (Module(m), module_path(Module(m))))
        .collect()
}

pub fn current_pid() -> u32 {
    // SAFETY: no preconditions.
    unsafe { GetCurrentProcessId() }
}
