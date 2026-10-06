//! IAT hook on `P_VirtualObject::setBoneMatrices`: the engine's call that
//! hands a character its bone matrices. The hook passes every call through
//! and records what the engine sends, so the argument's layout can be worked
//! out from the log.

use std::ffi::c_void;
use std::fmt::Write;
use std::sync::atomic::{AtomicUsize, Ordering};

use peimage::Import;
use winmod::log;

use crate::rv;
use crate::stats::HookStats;

pub const E2MFC: &str = "e2mfc.dll";
const IMPORT_PREFIX: &str = "?setBoneMatrices@P_VirtualObject@@";
/// Calls logged in detail from the start, then only first sightings of objects.
const DETAILED_CALLS: u32 = 16;
const DETAILED_OBJECTS: u32 = 64;

/// `void P_VirtualObject::setBoneMatrices(const std::rv<const M_Matrix4x3Template<float>*>&)`,
/// as the mangled name spells it: public, non-virtual, `__thiscall`. The
/// `-unwind` variant has the same calling convention; it lets a C++ exception
/// from the engine pass through the hook to the game's handler, which a plain
/// `"thiscall"` declaration makes undefined behavior.
type SetBoneMatrices = unsafe extern "thiscall-unwind" fn(this: *mut c_void, matrices: *const c_void);

static ORIGINAL: AtomicUsize = AtomicUsize::new(0);
pub static STATS: HookStats = HookStats::new();

/// Hooks the executable's import of `setBoneMatrices`; returns whether the
/// hook is in place.
pub fn install(exe_imports: &[Import]) -> bool {
    let from_e2mfc = |import: &&Import| import.dll.eq_ignore_ascii_case(E2MFC);
    let matches: Vec<&Import> = exe_imports
        .iter()
        .filter(from_e2mfc)
        .filter(|import| {
            import
                .name
                .as_deref()
                .is_some_and(|name| name.starts_with(IMPORT_PREFIX))
        })
        .collect();
    let import = match matches.as_slice() {
        [import] => *import,
        [] => {
            log!("no {E2MFC} import starts with {IMPORT_PREFIX}; {E2MFC} imports mentioning Bone:");
            for import in exe_imports.iter().filter(from_e2mfc) {
                if import.name.as_deref().is_some_and(|name| name.contains("Bone")) {
                    log!("  {import}");
                }
            }
            return false;
        }
        several => {
            // Overloads would have other signatures than the hook's.
            log!("{} imports start with {IMPORT_PREFIX}; not hooking any:", several.len());
            for import in several {
                log!("  {import}");
            }
            return false;
        }
    };

    // SAFETY: the slot is a pointer-sized IAT entry of the loaded executable.
    let original = unsafe { (import.slot as *const usize).read_volatile() };
    if original == 0 {
        log!("IAT slot {:#010x} of {import} is empty; not hooking", import.slot);
        return false;
    }
    // A game thread may enter the hook as soon as the slot changes.
    ORIGINAL.store(original, Ordering::Release);
    let hook = set_bone_matrices as SetBoneMatrices as usize;
    // SAFETY: the slot comes from Image::imports of the executable and the
    // hook has the signature the import's mangled name spells.
    match unsafe { peimage::patch_slot(import.slot, hook) } {
        Ok(previous) if previous == original => {
            log!("hooked {import}: slot {:#010x}, original {original:#010x}", import.slot);
        }
        Ok(previous) => {
            // Someone patched the slot in between; chain to what the game would
            // have called.
            ORIGINAL.store(previous, Ordering::Release);
            log!(
                "hooked {import}, but the slot {:#010x} changed from {original:#010x} to {previous:#010x} meanwhile",
                import.slot
            );
        }
        Err(err) => {
            log!("cannot patch the IAT slot of {import}: {err}");
            return false;
        }
    }
    true
}

unsafe extern "thiscall-unwind" fn set_bone_matrices(this: *mut c_void, matrices: *const c_void) {
    // SAFETY: ORIGINAL holds the function the slot pointed to; it was stored
    // before the hook became reachable.
    unsafe {
        let original = std::mem::transmute::<usize, SetBoneMatrices>(ORIGINAL.load(Ordering::Acquire));
        original(this, matrices);
    }
    let (this, arg) = (this as usize, matrices as usize);
    // A panic must not unwind into the game; winmod's panic hook logs it.
    let _ = std::panic::catch_unwind(|| record(this, arg));
}

fn record(this: usize, arg: usize) {
    let words = rv::read_words(arg, &read_memory);
    let call = STATS.record(this, words.as_ref());
    let new_object = call.first_sighting.filter(|&index| index < DETAILED_OBJECTS);
    if call.number >= DETAILED_CALLS && new_object.is_none() {
        return;
    }
    let mut text = format!("setBoneMatrices call {}", call.number);
    if let Some(index) = new_object {
        let _ = write!(text, ", first call on object {index}");
    }
    let _ = write!(text, ": this={this:#010x} arg={arg:#010x}");
    match words {
        Some(words) => log!("{text}\n{}", rv::describe(&words, &read_memory)),
        None => log!("{text}, argument unreadable"),
    }
}

/// Copies memory that is committed and readable right now. The argument and
/// what it owns stay alive during the call; a wrong interpretation may point
/// into memory another thread frees in between, which the probe accepts.
fn read_memory(addr: usize, buf: &mut [u8]) -> bool {
    if !winmod::readable(addr, buf.len()) {
        return false;
    }
    // SAFETY: the range was just confirmed readable.
    unsafe { std::ptr::copy_nonoverlapping(addr as *const u8, buf.as_mut_ptr(), buf.len()) };
    true
}
