//! The ped body bones the plugin streams: the source skeleton of the bridge.

/// A ped bone: its `ePedBone` name and tag from the SDK's `enums.h`, and the
/// index of its parent in [`BODY_BONES`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyBone {
    pub name: &'static str,
    pub tag: u16,
    pub parent: Option<usize>,
}

const fn bone(name: &'static str, tag: u16, parent: Option<usize>) -> BodyBone {
    BodyBone { name, tag, parent }
}

/// MP3 bones of the published poses, in their order; `Command::Hit::bone`
/// indexes this table too. Parents precede their children.
///
/// The SDK lists tags, not a hierarchy. The parents are an assumption taken
/// from the standard RAGE ped rig, to verify against the player skeleton dump:
/// PELVIS under ROOT, the spine chain up to the head, clavicles under SPINE3,
/// thighs under PELVIS.
pub const BODY_BONES: [BodyBone; 24] = [
    bone("ROOT", 0x0000, None),
    bone("PELVIS", 0xC018, Some(0)),
    bone("SPINE", 0x3A8D, Some(1)),
    bone("SPINE1", 0x0C9B, Some(2)),
    bone("SPINE2", 0x0C9C, Some(3)),
    bone("SPINE3", 0x0C9D, Some(4)),
    bone("NECK", 0xE168, Some(5)),
    bone("HEAD", 0xFFD0, Some(6)),
    bone("L_CLAVICLE", 0xC1AD, Some(5)),
    bone("R_CLAVICLE", 0xA1AD, Some(5)),
    bone("L_UPPERARM", 0x89A0, Some(8)),
    bone("R_UPPERARM", 0xA9A0, Some(9)),
    bone("L_FOREARM", 0x3AC6, Some(10)),
    bone("R_FOREARM", 0x3CC6, Some(11)),
    bone("L_HAND", 0x62DE, Some(12)),
    bone("R_HAND", 0x34BE, Some(13)),
    bone("L_THIGH", 0xA555, Some(1)),
    bone("R_THIGH", 0xBF02, Some(1)),
    bone("L_CALF", 0x1431, Some(16)),
    bone("R_CALF", 0xE4A0, Some(17)),
    bone("L_FOOT", 0xD4FE, Some(18)),
    bone("R_FOOT", 0xA6DE, Some(19)),
    bone("L_TOE", 0xB1C9, Some(20)),
    bone("R_TOE", 0x83A9, Some(21)),
];

pub fn body_bone_index(name: &str) -> Option<usize> {
    BODY_BONES.iter().position(|bone| bone.name == name)
}

#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    use super::*;

    #[test]
    fn parents_precede_children() {
        assert_eq!(BODY_BONES[0].parent, None);
        for (index, bone) in BODY_BONES.iter().enumerate().skip(1) {
            let parent = bone.parent.unwrap_or_else(|| panic!("{} has no parent", bone.name));
            assert!(parent < index, "{} comes before its parent", bone.name);
        }
    }

    #[test]
    fn names_and_tags_are_unique() {
        let names: HashSet<_> = BODY_BONES.iter().map(|bone| bone.name).collect();
        let tags: HashSet<_> = BODY_BONES.iter().map(|bone| bone.tag).collect();
        assert_eq!((names.len(), tags.len()), (BODY_BONES.len(), BODY_BONES.len()));
        // enums.h marks bones without a known tag with 0xFFFF.
        assert!(!tags.contains(&0xFFFF));
    }

    #[test]
    fn limbs_hang_from_their_own_side() {
        for bone in &BODY_BONES[1..] {
            let parent = BODY_BONES[bone.parent.unwrap()].name;
            for side in ["L_", "R_"] {
                if bone.name.starts_with(side) && !bone.name.ends_with("CLAVICLE") && !bone.name.ends_with("THIGH") {
                    assert!(parent.starts_with(side), "{} hangs from {parent}", bone.name);
                }
            }
        }
        assert_eq!(body_bone_index("PELVIS"), Some(1));
        assert_eq!(body_bone_index("R_TOE"), Some(BODY_BONES.len() - 1));
        assert_eq!(body_bone_index("TAIL"), None);
    }
}
