//! Exports and imports of a PE module as mapped by the Windows loader.
//!
//! Everything works on the in-memory image: an RVA is an offset from the
//! module base. Callers pass the base of a module that is loaded (or a test
//! buffer laid out the same way) and must keep it mapped while reading.

use std::ffi::CStr;

const DOS_MAGIC: u16 = 0x5A4D;
const NT_SIGNATURE: u32 = 0x0000_4550;
const PE32_MAGIC: u16 = 0x10B;
const PE32_PLUS_MAGIC: u16 = 0x20B;
const EXPORT_DIRECTORY: usize = 0;
const IMPORT_DIRECTORY: usize = 1;

#[derive(Clone, Debug, PartialEq)]
pub struct Export {
    pub name: String,
    pub ordinal: u32,
    pub rva: u32,
}

/// One imported function and the address of its IAT slot.
#[derive(Clone, Debug, PartialEq)]
pub struct Import {
    pub dll: String,
    /// `None` for imports by ordinal.
    pub name: Option<String>,
    pub slot: usize,
}

#[derive(Debug, PartialEq)]
pub enum Error {
    BadDosHeader,
    BadNtHeader,
    UnknownOptionalHeader(u16),
}

pub struct Image {
    base: usize,
    is64: bool,
    data_dirs: usize,
    dir_count: u32,
}

impl Image {
    /// # Safety
    /// `base` must point to a mapped PE image that stays mapped while the
    /// returned `Image` is used.
    pub unsafe fn new(base: usize) -> Result<Self, Error> {
        let image = Self {
            base,
            is64: false,
            data_dirs: 0,
            dir_count: 0,
        };
        if image.u16(0) != DOS_MAGIC {
            return Err(Error::BadDosHeader);
        }
        let nt = image.u32(0x3C) as usize;
        if image.u32(nt) != NT_SIGNATURE {
            return Err(Error::BadNtHeader);
        }
        let optional = nt + 24;
        let (is64, dir_count_at, data_dirs) = match image.u16(optional) {
            PE32_MAGIC => (false, optional + 92, optional + 96),
            PE32_PLUS_MAGIC => (true, optional + 108, optional + 112),
            other => return Err(Error::UnknownOptionalHeader(other)),
        };
        let dir_count = image.u32(dir_count_at);
        Ok(Self {
            base,
            is64,
            data_dirs,
            dir_count,
        })
    }

    pub fn base(&self) -> usize {
        self.base
    }

    fn u16(&self, rva: usize) -> u16 {
        // SAFETY: Image::new's contract; the PE fields read here lie inside the image.
        unsafe { ((self.base + rva) as *const u16).read_unaligned() }
    }

    fn u32(&self, rva: usize) -> u32 {
        // SAFETY: as above.
        unsafe { ((self.base + rva) as *const u32).read_unaligned() }
    }

    fn u64(&self, rva: usize) -> u64 {
        // SAFETY: as above.
        unsafe { ((self.base + rva) as *const u64).read_unaligned() }
    }

    fn cstr(&self, rva: u32) -> String {
        // SAFETY: as above; PE name fields are NUL-terminated.
        unsafe { CStr::from_ptr((self.base + rva as usize) as *const std::ffi::c_char) }
            .to_string_lossy()
            .into_owned()
    }

    fn directory(&self, index: usize) -> Option<(u32, u32)> {
        if index as u32 >= self.dir_count {
            return None;
        }
        let at = self.data_dirs + index * 8;
        let (rva, size) = (self.u32(at), self.u32(at + 4));
        (rva != 0).then_some((rva, size))
    }

    /// Named and ordinal-only exports; forwarders are skipped.
    pub fn exports(&self) -> Vec<Export> {
        let Some((dir, size)) = self.directory(EXPORT_DIRECTORY) else {
            return Vec::new();
        };
        let dir = dir as usize;
        let base_ordinal = self.u32(dir + 16);
        let function_count = self.u32(dir + 20) as usize;
        let name_count = self.u32(dir + 24) as usize;
        let functions = self.u32(dir + 28) as usize;
        let names = self.u32(dir + 32) as usize;
        let ordinals = self.u32(dir + 36) as usize;

        let mut names_by_index = vec![None; function_count];
        for i in 0..name_count {
            let index = self.u16(ordinals + 2 * i) as usize;
            if index < function_count {
                names_by_index[index] = Some(self.cstr(self.u32(names + 4 * i)));
            }
        }
        let forwarder = |rva: u32| rva as usize >= dir && (rva as usize) < dir + size as usize;
        (0..function_count)
            .filter_map(|index| {
                let rva = self.u32(functions + 4 * index);
                if rva == 0 || forwarder(rva) {
                    return None;
                }
                let ordinal = base_ordinal + index as u32;
                let name = names_by_index[index].take().unwrap_or_else(|| format!("#{ordinal}"));
                Some(Export { name, ordinal, rva })
            })
            .collect()
    }

    /// Every import with the address of the IAT slot the loader filled in.
    pub fn imports(&self) -> Vec<Import> {
        let Some((dir, _)) = self.directory(IMPORT_DIRECTORY) else {
            return Vec::new();
        };
        let thunk_size = if self.is64 { 8 } else { 4 };
        let ordinal_flag = if self.is64 { 1u64 << 63 } else { 1u64 << 31 };
        let mut result = Vec::new();
        let mut descriptor = dir as usize;
        loop {
            let lookup = self.u32(descriptor);
            let name = self.u32(descriptor + 12);
            let iat = self.u32(descriptor + 16);
            if name == 0 && iat == 0 {
                break;
            }
            let dll = self.cstr(name);
            // Bound or lookup-less images only have the IAT; names are then lost
            // once the loader overwrites it, so prefer the lookup table.
            let names_table = if lookup != 0 { lookup } else { iat } as usize;
            for i in 0.. {
                let at = names_table + i * thunk_size;
                let entry = if self.is64 { self.u64(at) } else { self.u32(at) as u64 };
                if entry == 0 {
                    break;
                }
                let name = (entry & ordinal_flag == 0).then(|| self.cstr((entry & 0x7FFF_FFFF) as u32 + 2));
                result.push(Import {
                    dll: dll.clone(),
                    name,
                    slot: self.base + iat as usize + i * thunk_size,
                });
            }
            descriptor += 20;
        }
        result
    }
}

/// Replaces the function pointer in an IAT slot, returning the previous one.
///
/// # Safety
/// `slot` must be an IAT slot from [`Image::imports`] of a loaded module and
/// `replacement` a function with the same signature and calling convention.
#[cfg(windows)]
pub unsafe fn patch_slot(slot: usize, replacement: usize) -> std::io::Result<usize> {
    use windows_sys::Win32::System::Memory::{PAGE_PROTECTION_FLAGS, PAGE_READWRITE, VirtualProtect};

    let size = size_of::<usize>();
    let mut old: PAGE_PROTECTION_FLAGS = 0;
    // SAFETY: slot is a valid pointer-sized location in a loaded image.
    unsafe {
        if VirtualProtect(slot as *const _, size, PAGE_READWRITE, &mut old) == 0 {
            return Err(std::io::Error::last_os_error());
        }
        let previous = (slot as *mut usize).replace(replacement);
        let mut ignored: PAGE_PROTECTION_FLAGS = 0;
        VirtualProtect(slot as *const _, size, old, &mut ignored);
        Ok(previous)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds a mapped PE32 image: exports `?b@C@@QAEXXZ`, `a`, one ordinal-only
    /// function and a forwarder; imports two names and one ordinal from
    /// `e2mfc.dll` plus one name from `kernel32.dll`.
    struct Builder(Vec<u8>);

    impl Builder {
        fn put_u16(&mut self, at: usize, v: u16) {
            self.0[at..at + 2].copy_from_slice(&v.to_le_bytes());
        }
        fn put_u32(&mut self, at: usize, v: u32) {
            self.0[at..at + 4].copy_from_slice(&v.to_le_bytes());
        }
        fn put_str(&mut self, at: usize, s: &str) {
            self.0[at..at + s.len()].copy_from_slice(s.as_bytes());
        }
    }

    fn build_image() -> Vec<u8> {
        let mut b = Builder(vec![0u8; 0x3000]);
        b.put_u16(0, DOS_MAGIC);
        b.put_u32(0x3C, 0x80);
        b.put_u32(0x80, NT_SIGNATURE);
        let optional = 0x80 + 24;
        b.put_u16(optional, PE32_MAGIC);
        b.put_u32(optional + 92, 16);
        let dirs = optional + 96;
        b.put_u32(dirs, 0x1000); // export directory
        b.put_u32(dirs + 4, 0x200);
        b.put_u32(dirs + 8, 0x2000); // import directory
        b.put_u32(dirs + 12, 0x100);

        // Export directory: ordinal base 5, four functions, two names.
        let e = 0x1000;
        b.put_u32(e + 16, 5);
        b.put_u32(e + 20, 4);
        b.put_u32(e + 24, 2);
        b.put_u32(e + 28, 0x1100); // functions
        b.put_u32(e + 32, 0x1140); // names
        b.put_u32(e + 36, 0x1160); // name ordinals
        b.put_u32(0x1100, 0x1800); // index 0: named "a"
        b.put_u32(0x1104, 0x1810); // index 1: ordinal-only
        b.put_u32(0x1108, 0x1180); // index 2: forwarder (inside export dir)
        b.put_u32(0x110C, 0x1820); // index 3: named C++ member
        b.put_u32(0x1140, 0x11C0);
        b.put_u32(0x1144, 0x11D0);
        b.put_u16(0x1160, 3);
        b.put_u16(0x1162, 0);
        b.put_str(0x1180, "other.dll.f");
        b.put_str(0x11C0, "?b@C@@QAEXXZ");
        b.put_str(0x11D0, "a");

        // Import descriptors: e2mfc.dll, kernel32.dll, terminator.
        let i = 0x2000;
        b.put_u32(i, 0x2100); // lookup table
        b.put_u32(i + 12, 0x2300); // dll name
        b.put_u32(i + 16, 0x2200); // IAT
        b.put_u32(i + 20, 0x2120);
        b.put_u32(i + 32, 0x2310);
        b.put_u32(i + 36, 0x2220);
        b.put_u32(0x2100, 0x2400);
        b.put_u32(0x2104, 0x8000_0007);
        b.put_u32(0x2108, 0x2440);
        b.put_u32(0x2120, 0x2480);
        b.put_str(0x2300, "e2mfc.dll");
        b.put_str(0x2310, "KERNEL32.dll");
        b.put_str(0x2402, "?setBoneMatrices@P_VirtualObject@@QAEXXZ");
        b.put_str(0x2442, "?getCount@P_Scene@@QBEHXZ");
        b.put_str(0x2482, "Sleep");
        // The loader overwrote the IAT with resolved addresses.
        b.put_u32(0x2200, 0x7700_0001);
        b.put_u32(0x2204, 0x7700_0002);
        b.put_u32(0x2208, 0x7700_0003);
        b.put_u32(0x2220, 0x7600_0001);
        b.0
    }

    #[test]
    fn reads_exports() {
        let image_bytes = build_image();
        let image = unsafe { Image::new(image_bytes.as_ptr() as usize) }.unwrap();
        let exports = image.exports();
        assert_eq!(
            exports,
            vec![
                Export {
                    name: "a".into(),
                    ordinal: 5,
                    rva: 0x1800
                },
                Export {
                    name: "#6".into(),
                    ordinal: 6,
                    rva: 0x1810
                },
                Export {
                    name: "?b@C@@QAEXXZ".into(),
                    ordinal: 8,
                    rva: 0x1820
                },
            ]
        );
    }

    #[test]
    fn reads_imports_with_slots() {
        let image_bytes = build_image();
        let base = image_bytes.as_ptr() as usize;
        let image = unsafe { Image::new(base) }.unwrap();
        let imports = image.imports();
        let names: Vec<_> = imports
            .iter()
            .map(|i| (i.dll.as_str(), i.name.as_deref(), i.slot - base))
            .collect();
        assert_eq!(
            names,
            vec![
                ("e2mfc.dll", Some("?setBoneMatrices@P_VirtualObject@@QAEXXZ"), 0x2200),
                ("e2mfc.dll", None, 0x2204),
                ("e2mfc.dll", Some("?getCount@P_Scene@@QBEHXZ"), 0x2208),
                ("KERNEL32.dll", Some("Sleep"), 0x2220),
            ]
        );
        let slot = imports[0].slot as *const u32;
        assert_eq!(unsafe { slot.read_unaligned() }, 0x7700_0001);
    }

    #[test]
    fn rejects_non_pe() {
        let bytes = vec![0u8; 0x200];
        assert_eq!(
            unsafe { Image::new(bytes.as_ptr() as usize) }.err(),
            Some(Error::BadDosHeader)
        );
    }
}
