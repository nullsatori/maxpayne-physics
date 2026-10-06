//! Named Windows file mapping backed by the page file.

use std::ptr::NonNull;

use windows_sys::Win32::Foundation::{CloseHandle, GetLastError, HANDLE, INVALID_HANDLE_VALUE};
use windows_sys::Win32::System::Memory::{
    CreateFileMappingW, FILE_MAP_ALL_ACCESS, MEMORY_MAPPED_VIEW_ADDRESS, MapViewOfFile, PAGE_READWRITE, UnmapViewOfFile,
};

use crate::{OpenError, Shared};

pub struct Mapping {
    handle: HANDLE,
    view: NonNull<Shared>,
}

impl Mapping {
    /// Creates the mapping or opens the one the other game already created.
    /// A new mapping is zero-filled by the OS.
    pub fn open(name: &str, size: usize) -> Result<Self, OpenError> {
        let wide: Vec<u16> = name.encode_utf16().chain(Some(0)).collect();
        // SAFETY: plain Win32 calls with valid arguments; results are checked.
        unsafe {
            let handle = CreateFileMappingW(
                INVALID_HANDLE_VALUE,
                std::ptr::null(),
                PAGE_READWRITE,
                0,
                size as u32,
                wide.as_ptr(),
            );
            if handle.is_null() {
                return Err(OpenError::Os {
                    call: "CreateFileMappingW",
                    code: GetLastError(),
                });
            }
            let view = MapViewOfFile(handle, FILE_MAP_ALL_ACCESS, 0, 0, size);
            let Some(view) = NonNull::new(view.Value as *mut Shared) else {
                let code = GetLastError();
                CloseHandle(handle);
                return Err(OpenError::Os {
                    call: "MapViewOfFile",
                    code,
                });
            };
            Ok(Self { handle, view })
        }
    }

    pub fn view(&self) -> NonNull<Shared> {
        self.view
    }
}

impl Drop for Mapping {
    fn drop(&mut self) {
        // SAFETY: view and handle were obtained in `open` and are released once.
        unsafe {
            UnmapViewOfFile(MEMORY_MAPPED_VIEW_ADDRESS {
                Value: self.view.as_ptr().cast(),
            });
            CloseHandle(self.handle);
        }
    }
}
