//! Shared plumbing for DLLs injected by an ASI loader. Windows only.
#![cfg(windows)]

use std::ffi::{OsString, c_void};
use std::fmt;
use std::fs::File;
use std::io::Write;
use std::os::windows::ffi::OsStringExt;
use std::path::{Path, PathBuf};
use std::ptr::{null, null_mut};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use windows_sys::Win32::Foundation::{CloseHandle, MAX_PATH};
use windows_sys::Win32::System::LibraryLoader::{DisableThreadLibraryCalls, GetModuleFileNameW, GetModuleHandleW};
use windows_sys::Win32::System::Memory::{
    MEM_COMMIT, MEMORY_BASIC_INFORMATION, PAGE_EXECUTE_READ, PAGE_EXECUTE_READWRITE, PAGE_EXECUTE_WRITECOPY,
    PAGE_GUARD, PAGE_NOACCESS, PAGE_READONLY, PAGE_READWRITE, PAGE_WRITECOPY, VirtualQuery,
};
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

unsafe extern "system" fn thread_main(param: *mut c_void) -> u32 {
    // SAFETY: `param` is the Box leaked in on_attach, consumed exactly once.
    let start = unsafe { Box::from_raw(param as *mut Start) };
    std::panic::set_hook(Box::new(|info| write_log(format_args!("PANIC: {info}"))));
    // A panic must not unwind into the game; the hook above records it.
    match std::panic::catch_unwind(|| (start.init)(start.module)) {
        Ok(()) => 0,
        Err(_) => 1,
    }
}

/// `<dll name>.log` next to the injected DLL.
pub fn log_path(module: Module) -> PathBuf {
    side_path(module, "log")
}

/// `<dll name>.<extension>` next to the injected DLL.
pub fn side_path(module: Module, extension: &str) -> PathBuf {
    module_path(module).with_extension(extension)
}

/// Writes `items`, one per line, to `<dll name>.<extension>` next to the
/// injected DLL and logs the outcome.
pub fn write_listing<T: fmt::Display>(module: Module, extension: &str, items: &[T]) {
    let path = side_path(module, extension);
    let text: String = items.iter().map(|item| format!("{item}\n")).collect();
    match std::fs::write(&path, text) {
        Ok(()) => write_log(format_args!("wrote {} lines to {}", items.len(), path.display())),
        Err(err) => write_log(format_args!("cannot write {}: {err}", path.display())),
    }
}

struct Log {
    file: File,
    start: Instant,
}

static LOG: Mutex<Option<Log>> = Mutex::new(None);

/// Opens the process-wide log; later calls replace it.
pub fn init_log(path: &Path) {
    let file = File::create(path).ok();
    if let Ok(mut log) = LOG.lock() {
        *log = file.map(|file| Log {
            file,
            start: Instant::now(),
        });
    }
}

/// Writes one line with seconds since `init_log`; flushed so it survives a crash.
/// Safe to call from game threads; does nothing before `init_log`.
pub fn write_log(args: fmt::Arguments) {
    if let Ok(mut guard) = LOG.lock()
        && let Some(log) = guard.as_mut()
    {
        let _ = writeln!(log.file, "[{:9.3}] {args}", log.start.elapsed().as_secs_f64());
        let _ = log.file.flush();
    }
}

#[macro_export]
macro_rules! log {
    ($($arg:tt)*) => { $crate::write_log(format_args!($($arg)*)) };
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

/// Handle of a module already loaded in this process; `None` for the executable.
pub fn module_handle(name: Option<&str>) -> Option<Module> {
    let wide: Option<Vec<u16>> = name.map(|n| n.encode_utf16().chain(Some(0)).collect());
    let ptr = wide.as_ref().map_or(null(), |w| w.as_ptr());
    // SAFETY: ptr is null or a NUL-terminated UTF-16 string.
    let handle = unsafe { GetModuleHandleW(ptr) };
    (!handle.is_null()).then_some(Module(handle))
}

/// Polls every 100 ms until the named module is loaded, for at most `timeout`.
pub fn wait_for_module(name: &str, timeout: Duration) -> Option<Module> {
    let start = Instant::now();
    loop {
        if let Some(module) = module_handle(Some(name)) {
            return Some(module);
        }
        if start.elapsed() >= timeout {
            return None;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}

/// Whether `len` bytes at `addr` are committed, readable memory right now.
pub fn readable(addr: usize, len: usize) -> bool {
    let mut info: MEMORY_BASIC_INFORMATION = unsafe { std::mem::zeroed() };
    // SAFETY: VirtualQuery accepts any address and fills `info`.
    let filled = unsafe { VirtualQuery(addr as *const c_void, &mut info, size_of::<MEMORY_BASIC_INFORMATION>()) };
    if filled == 0 || info.State != MEM_COMMIT || info.Protect & (PAGE_GUARD | PAGE_NOACCESS) != 0 {
        return false;
    }
    let readable_flags = PAGE_READONLY
        | PAGE_READWRITE
        | PAGE_WRITECOPY
        | PAGE_EXECUTE_READ
        | PAGE_EXECUTE_READWRITE
        | PAGE_EXECUTE_WRITECOPY;
    let region_end = info.BaseAddress as usize + info.RegionSize;
    info.Protect & readable_flags != 0 && addr.checked_add(len).is_some_and(|end| end <= region_end)
}

pub fn current_pid() -> u32 {
    // SAFETY: no preconditions.
    unsafe { GetCurrentProcessId() }
}
