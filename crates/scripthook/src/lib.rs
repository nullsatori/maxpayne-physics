//! Script Hook for Max Payne 3 (`ScriptHook.dll`), bound at run time.
//!
//! The plugin links no import library: [`Api::bind`] finds the few C++
//! functions it calls among the DLL's exports by their MSVC-decorated names
//! and checks each decoration against the SDK declaration, so a Script Hook
//! whose functions have other signatures is refused rather than called.
//!
//! Natives are called the SDK's way (`nativeInit`, one `nativePush32` per
//! 32-bit argument, `nativeCall`) through the [`Backend`] seam; tests put a
//! [`Recorder`] in place of Script Hook.
//!
//! Declarations, native hashes and bone tags come from the public plugin SDK,
//! github.com/alloc8or/mp3-plugin-sdk (`script/main.h`, `natives.h`,
//! `enums.h`). Only the ones used here are copied, each where it is used.

mod api;
mod bones;
mod native;

pub use api::{Api, BindError, Script, ScriptEntry};
pub use bones::{BODY_BONES, BodyBone, body_bone_index};
pub use native::{Backend, Hash, Natives, Ped, Player, Recorder};
