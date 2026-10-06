//! The Script Hook functions the plugin calls, found among the DLL's exports.
//!
//! `main.h` declares them as plain C++ functions, so they are exported under
//! MSVC-decorated names: `?name@@`, then `Y` (a global function) and the
//! calling convention (`A` __cdecl, `G` __stdcall, `I` __fastcall), then the
//! return type, the parameters and a terminator (`@Z` after parameters, `XZ`
//! for none). Type codes used below: `X` void, `I` unsigned int (uint32_t),
//! `K` unsigned long (DWORD, Hash), `PAI` uint32_t*, `PAUHINSTANCE__@@`
//! HMODULE, `P6AXXZ` a pointer to a __cdecl function taking and returning
//! nothing. The SDK's import library `lib/ScriptHook.lib` carries the same
//! five names.

use std::ffi::c_void;
use std::fmt;
use std::marker::PhantomData;

use peimage::Export;

use crate::native::{Backend, Hash};

/// A script's main function. Script Hook runs it on a fiber of the game's
/// script thread; it must never unwind into Script Hook.
pub type ScriptEntry = extern "C" fn();

// cdecl on x86, which is what the `YA` in each decoration promises.
type ScriptRegister = unsafe extern "C" fn(module: *mut c_void, entry: ScriptEntry);
type ScriptWait = unsafe extern "C" fn(ms: u32);
type NativeInit = unsafe extern "C" fn(hash: Hash);
type NativePush32 = unsafe extern "C" fn(value: u32);
type NativeCall = unsafe extern "C" fn() -> *mut u32;

/// A function as `main.h` declares it.
struct Decl {
    name: &'static str,
    signature: &'static str,
    /// Return type and parameter codes with the terminator, after `YA`.
    types: &'static str,
}

const CDECL: &str = "YA";

const SCRIPT_REGISTER: Decl = Decl {
    name: "scriptRegister",
    signature: "void scriptRegister(HMODULE module, void(*function)())",
    types: "XPAUHINSTANCE__@@P6AXXZ@Z",
};
const SCRIPT_WAIT: Decl = Decl {
    name: "scriptWait",
    signature: "void scriptWait(DWORD ms)",
    types: "XK@Z",
};
const NATIVE_INIT: Decl = Decl {
    name: "nativeInit",
    signature: "void nativeInit(Hash hash)",
    types: "XK@Z",
};
const NATIVE_PUSH32: Decl = Decl {
    name: "nativePush32",
    signature: "void nativePush32(uint32_t value)",
    types: "XI@Z",
};
const NATIVE_CALL: Decl = Decl {
    name: "nativeCall",
    signature: "uint32_t* nativeCall()",
    types: "PAIXZ",
};

impl Decl {
    fn prefix(&self) -> String {
        format!("?{}@@", self.name)
    }

    fn decorated(&self) -> String {
        format!("?{}@@{CDECL}{}", self.name, self.types)
    }

    /// Why `found`, an export with this function's prefix, is not it.
    fn mismatch(&self, found: &str) -> String {
        let rest = &found[self.prefix().len()..];
        let convention = rest.get(..2).unwrap_or(rest);
        let types = rest.get(2..).unwrap_or_default();
        let mut reasons = Vec::new();
        if convention != CDECL {
            reasons.push(format!("calling convention code {convention}, not {CDECL} (__cdecl)"));
        }
        if types != self.types {
            reasons.push(format!("return and parameter codes {types}, not {}", self.types));
        }
        format!("{found} ({})", reasons.join("; "))
    }
}

#[derive(Debug, PartialEq)]
pub enum BindError {
    /// No export starts with `?name@@`.
    Missing { expected: String, signature: &'static str },
    /// Exports start with `?name@@`, but none is decorated as the SDK declares
    /// the function; each entry names one and says how it differs.
    Mismatch {
        expected: String,
        signature: &'static str,
        found: Vec<String>,
    },
}

impl fmt::Display for BindError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BindError::Missing { expected, signature } => {
                write!(f, "no export for `{signature}`: expected {expected}")
            }
            BindError::Mismatch {
                expected,
                signature,
                found,
            } => write!(
                f,
                "no export matches `{signature}`: expected {expected}, found {}",
                found.join(", ")
            ),
        }
    }
}

/// The RVA of the export decorated exactly as `decl`.
fn find(exports: &[Export], decl: &Decl) -> Result<u32, BindError> {
    let prefix = decl.prefix();
    let expected = decl.decorated();
    let candidates: Vec<&Export> = exports.iter().filter(|e| e.name.starts_with(&prefix)).collect();
    // Another overload next to the declared one would be harmless.
    if let Some(export) = candidates.iter().find(|e| e.name == expected) {
        return Ok(export.rva);
    }
    let signature = decl.signature;
    if candidates.is_empty() {
        return Err(BindError::Missing { expected, signature });
    }
    Err(BindError::Mismatch {
        expected,
        signature,
        found: candidates.iter().map(|e| decl.mismatch(&e.name)).collect(),
    })
}

/// The Script Hook functions the plugin calls.
#[derive(Clone, Copy, Debug)]
pub struct Api {
    register: ScriptRegister,
    wait: ScriptWait,
    init: NativeInit,
    push32: NativePush32,
    call: NativeCall,
}

impl Api {
    /// Binds the functions from the exports of the module loaded at `base`.
    ///
    /// # Safety
    /// `exports` must be the exports of the image at `base`, and the image
    /// must stay loaded while the `Api` is used.
    pub unsafe fn bind(exports: &[Export], base: usize) -> Result<Self, BindError> {
        let address = |decl: &Decl| find(exports, decl).map(|rva| base + rva as usize);
        let (register, wait, init, push32, call) = (
            address(&SCRIPT_REGISTER)?,
            address(&SCRIPT_WAIT)?,
            address(&NATIVE_INIT)?,
            address(&NATIVE_PUSH32)?,
            address(&NATIVE_CALL)?,
        );
        // SAFETY: each address is an export whose decoration spells the
        // calling convention and signature of the pointer type it becomes.
        unsafe {
            Ok(Self {
                register: std::mem::transmute::<usize, ScriptRegister>(register),
                wait: std::mem::transmute::<usize, ScriptWait>(wait),
                init: std::mem::transmute::<usize, NativeInit>(init),
                push32: std::mem::transmute::<usize, NativePush32>(push32),
                call: std::mem::transmute::<usize, NativeCall>(call),
            })
        }
    }

    /// `scriptRegister`: Script Hook will run `entry` as a script of `module`.
    ///
    /// # Safety
    /// `module` must be the base address of the calling DLL, which stays
    /// loaded; `entry` must catch its panics.
    pub unsafe fn register(&self, module: usize, entry: ScriptEntry) {
        // SAFETY: bound by `bind`; arguments as the caller promised.
        unsafe { (self.register)(module as *mut c_void, entry) }
    }

    /// Natives and `scriptWait` for the script running on the current fiber.
    ///
    /// # Safety
    /// Only on the fiber of a script registered with [`Api::register`]: Script
    /// Hook runs natives and waits only there.
    pub unsafe fn script(&self) -> Script {
        Script {
            api: *self,
            _fiber: PhantomData,
        }
    }
}

/// Script Hook calls of a running script. Not `Send`: it stays on the fiber
/// it was made on.
pub struct Script {
    api: Api,
    _fiber: PhantomData<*const ()>,
}

impl Script {
    /// `scriptWait`: yields to the game; 0 resumes on the next frame.
    pub fn wait(&self, ms: u32) {
        // SAFETY: Script exists only on a script fiber (Api::script).
        unsafe { (self.api.wait)(ms) }
    }
}

// SAFETY: nativeCall returns Script Hook's storage for the result of the
// native just called; the SDK's `invoke` reads the result through it too.
unsafe impl Backend for Script {
    fn init(&mut self, hash: Hash) {
        // SAFETY: on a script fiber (Api::script).
        unsafe { (self.api.init)(hash) }
    }

    fn push(&mut self, value: u32) {
        // SAFETY: as above.
        unsafe { (self.api.push32)(value) }
    }

    fn call(&mut self) -> *const u32 {
        // SAFETY: as above.
        unsafe { (self.api.call)() }
    }
}

#[cfg(test)]
mod tests {
    use std::cell::{Cell, RefCell};

    use super::*;
    use crate::native::Natives;

    fn export(name: &str, rva: u32) -> Export {
        Export {
            name: name.to_owned(),
            ordinal: rva,
            rva,
        }
    }

    #[test]
    fn decorations_follow_the_declarations() {
        let names: Vec<String> = [SCRIPT_REGISTER, SCRIPT_WAIT, NATIVE_INIT, NATIVE_PUSH32, NATIVE_CALL]
            .iter()
            .map(Decl::decorated)
            .collect();
        assert_eq!(
            names,
            [
                "?scriptRegister@@YAXPAUHINSTANCE__@@P6AXXZ@Z",
                "?scriptWait@@YAXK@Z",
                "?nativeInit@@YAXK@Z",
                "?nativePush32@@YAXI@Z",
                "?nativeCall@@YAPAIXZ",
            ]
        );
    }

    #[test]
    fn finds_the_exact_decoration_among_overloads() {
        let exports = [
            export("?scriptWaitEx@@YAXK@Z", 0x10),
            export("?scriptWait@@YAXH@Z", 0x20),
            export("?scriptWait@@YAXK@Z", 0x30),
        ];
        assert_eq!(find(&exports, &SCRIPT_WAIT), Ok(0x30));
    }

    #[test]
    fn refuses_other_conventions_and_signatures() {
        let exports = [
            export("?scriptWaitEx@@YAXK@Z", 0x10),
            export("?nativeCall@@YGPAIXZ", 0x20),
            export("?nativePush32@@YAXK@Z", 0x30),
            export("?nativeInit@@YIXI@Z", 0x40),
        ];
        assert_eq!(
            find(&exports, &SCRIPT_WAIT),
            Err(BindError::Missing {
                expected: "?scriptWait@@YAXK@Z".into(),
                signature: SCRIPT_WAIT.signature,
            })
        );
        let error = find(&exports, &NATIVE_CALL).unwrap_err();
        assert_eq!(
            error.to_string(),
            "no export matches `uint32_t* nativeCall()`: expected ?nativeCall@@YAPAIXZ, found \
             ?nativeCall@@YGPAIXZ (calling convention code YG, not YA (__cdecl))"
        );
        assert_eq!(
            find(&exports, &NATIVE_PUSH32).unwrap_err().to_string(),
            "no export matches `void nativePush32(uint32_t value)`: expected ?nativePush32@@YAXI@Z, found \
             ?nativePush32@@YAXK@Z (return and parameter codes XK@Z, not XI@Z)"
        );
        assert_eq!(
            find(&exports, &NATIVE_INIT).unwrap_err().to_string(),
            "no export matches `void nativeInit(Hash hash)`: expected ?nativeInit@@YAXK@Z, found \
             ?nativeInit@@YIXI@Z (calling convention code YI, not YA (__cdecl); \
             return and parameter codes XI@Z, not XK@Z)"
        );
        // A truncated decoration is reported, not sliced past its end.
        let short = [export("?scriptWait@@Y", 0x50)];
        assert!(matches!(find(&short, &SCRIPT_WAIT), Err(BindError::Mismatch { .. })));
    }

    // Stand-ins for Script Hook's functions, recording what they receive.
    thread_local! {
        static SEEN: RefCell<Vec<String>> = const { RefCell::new(Vec::new()) };
        static RESULT: Cell<[u32; 3]> = const { Cell::new([7, 8, 9]) };
    }

    fn seen(event: String) {
        SEEN.with(|seen| seen.borrow_mut().push(event));
    }

    extern "C" fn entry() {}

    unsafe extern "C" fn fake_register(module: *mut c_void, script: ScriptEntry) {
        let is_entry = script as usize == entry as ScriptEntry as usize;
        seen(format!("register {:#x} entry={is_entry}", module as usize));
    }
    unsafe extern "C" fn fake_wait(ms: u32) {
        seen(format!("wait {ms}"));
    }
    unsafe extern "C" fn fake_init(hash: Hash) {
        seen(format!("init {hash:#010x}"));
    }
    unsafe extern "C" fn fake_push32(value: u32) {
        seen(format!("push {value:#x}"));
    }
    unsafe extern "C" fn fake_call() -> *mut u32 {
        seen("call".into());
        RESULT.with(|result| result.as_ptr().cast())
    }

    #[test]
    fn bound_functions_are_the_exports() {
        let addresses = [
            fake_register as ScriptRegister as usize,
            fake_wait as ScriptWait as usize,
            fake_init as NativeInit as usize,
            fake_push32 as NativePush32 as usize,
            fake_call as NativeCall as usize,
        ];
        let base = *addresses.iter().min().unwrap();
        let decls = [SCRIPT_REGISTER, SCRIPT_WAIT, NATIVE_INIT, NATIVE_PUSH32, NATIVE_CALL];
        let mut exports: Vec<Export> = decls
            .iter()
            .zip(addresses)
            .map(|(decl, address)| export(&decl.decorated(), u32::try_from(address - base).unwrap()))
            .collect();
        exports.push(export("?scriptUnregister@@YAXPAUHINSTANCE__@@@Z", 0));

        // SAFETY: the exports point at functions of the declared signatures.
        let api = unsafe { Api::bind(&exports, base) }.unwrap();
        unsafe { api.register(0x1000_0000, entry) };
        let mut natives = Natives::new(unsafe { api.script() });
        assert!(natives.does_ped_exist(-2));
        natives.backend().wait(0);
        let events = SEEN.with(|seen| seen.take());
        assert_eq!(
            events,
            [
                "register 0x10000000 entry=true",
                "init 0xf7c8d072",
                "push 0xfffffffe",
                "call",
                "wait 0",
            ]
        );
    }
}
