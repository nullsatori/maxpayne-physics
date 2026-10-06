use ragdoll::{BodyDef, FIXED_DT, JointDef, Ragdoll, RagdollWorld};
use rapier3d::prelude::*;

// Test humanoid in a T-pose, bones axis-aligned, feet at y = 0.
const PELVIS: usize = 0;
const SPINE: usize = 1;
const HEAD: usize = 2;
const L_UPPER_ARM: usize = 3;
const L_FOREARM: usize = 4;
const R_UPPER_ARM: usize = 5;
const R_FOREARM: usize = 6;
const L_THIGH: usize = 7;
const L_SHIN: usize = 8;
const R_THIGH: usize = 9;
const R_SHIN: usize = 10;

fn bind_pose() -> Vec<Pose> {
    [
        Vector::new(0.0, 0.95, 0.0),
        Vector::new(0.0, 1.15, 0.0),
        Vector::new(0.0, 1.55, 0.0),
        Vector::new(0.2, 1.45, 0.0),
        Vector::new(0.48, 1.45, 0.0),
        Vector::new(-0.2, 1.45, 0.0),
        Vector::new(-0.48, 1.45, 0.0),
        Vector::new(0.1, 0.9, 0.0),
        Vector::new(0.1, 0.48, 0.0),
        Vector::new(-0.1, 0.9, 0.0),
        Vector::new(-0.1, 0.48, 0.0),
    ]
    .into_iter()
    .map(Pose::from_translation)
    .collect()
}

// Knees bend around the lateral axis; arms lie along X, so elbows bend
// around Y. Limits are signed so every hinge folds toward +Z.
const KNEE: JointDef = JointDef::Hinge {
    axis: Vector::X,
    limits: [-2.4, 0.0],
};
const L_ELBOW: JointDef = JointDef::Hinge {
    axis: Vector::Y,
    limits: [-2.4, 0.0],
};
const R_ELBOW: JointDef = JointDef::Hinge {
    axis: Vector::Y,
    limits: [0.0, 2.4],
};

fn humanoid() -> Vec<BodyDef> {
    let ball = |swing: f32, twist: f32| JointDef::Ball {
        x: [-swing, swing],
        y: [-twist, twist],
        z: [-swing, swing],
    };
    let body = |bone, parent, end: Vector, radius| BodyDef {
        bone,
        parent,
        end,
        radius,
        density: 1000.0,
        joint: ball(0.5, 0.3),
    };
    vec![
        body(PELVIS, None, Vector::new(0.0, 0.18, 0.0), 0.13),
        body(SPINE, Some(0), Vector::new(0.0, 0.3, 0.0), 0.14),
        BodyDef {
            joint: ball(0.6, 0.8),
            ..body(HEAD, Some(1), Vector::new(0.0, 0.2, 0.0), 0.1)
        },
        BodyDef {
            joint: ball(1.4, 0.8),
            ..body(L_UPPER_ARM, Some(1), Vector::new(0.26, 0.0, 0.0), 0.05)
        },
        BodyDef {
            joint: L_ELBOW,
            ..body(L_FOREARM, Some(3), Vector::new(0.26, 0.0, 0.0), 0.045)
        },
        BodyDef {
            joint: ball(1.4, 0.8),
            ..body(R_UPPER_ARM, Some(1), Vector::new(-0.26, 0.0, 0.0), 0.05)
        },
        BodyDef {
            joint: R_ELBOW,
            ..body(R_FOREARM, Some(5), Vector::new(-0.26, 0.0, 0.0), 0.045)
        },
        BodyDef {
            joint: ball(1.2, 0.4),
            ..body(L_THIGH, Some(0), Vector::new(0.0, -0.4, 0.0), 0.075)
        },
        BodyDef {
            joint: KNEE,
            ..body(L_SHIN, Some(7), Vector::new(0.0, -0.42, 0.0), 0.06)
        },
        BodyDef {
            joint: ball(1.2, 0.4),
            ..body(R_THIGH, Some(0), Vector::new(0.0, -0.4, 0.0), 0.075)
        },
        BodyDef {
            joint: KNEE,
            ..body(R_SHIN, Some(9), Vector::new(0.0, -0.42, 0.0), 0.06)
        },
    ]
}

fn world_with_floor() -> RagdollWorld {
    let mut world = RagdollWorld::new();
    let s = 20.0;
    let vertices = vec![
        Vector::new(-s, 0.0, -s),
        Vector::new(s, 0.0, -s),
        Vector::new(s, 0.0, s),
        Vector::new(-s, 0.0, s),
    ];
    world.add_static_mesh(vertices, vec![[0, 2, 1], [0, 3, 2]]).unwrap();
    world
}

fn lifted(pose: &[Pose], offset: Vector) -> Vec<Pose> {
    pose.iter().map(|p| Pose::from_translation(offset) * *p).collect()
}

fn pose_of(ragdoll: &Ragdoll, world: &RagdollWorld, bone: usize) -> Pose {
    ragdoll
        .bone_poses(world)
        .into_iter()
        .find(|(b, _)| *b == bone)
        .unwrap()
        .1
}

/// Rotation of `child` relative to `parent`, measured from the bind pose.
fn joint_rotation(ragdoll: &Ragdoll, world: &RagdollWorld, bind: &[Pose], parent: usize, child: usize) -> Rotation {
    let rel_now = pose_of(ragdoll, world, parent).inverse() * pose_of(ragdoll, world, child);
    let rel_bind = bind[parent].inverse() * bind[child];
    rel_bind.rotation.inverse() * rel_now.rotation
}

fn run(world: &mut RagdollWorld, seconds: f32) {
    let frames = (seconds * 60.0).round() as u32;
    for _ in 0..frames {
        world.advance(1.0 / 60.0);
    }
}

#[test]
fn spawn_matches_input_pose() {
    let mut world = world_with_floor();
    let bind = bind_pose();
    let pose = lifted(&bind, Vector::new(3.0, 0.5, -2.0));
    let ragdoll = Ragdoll::spawn(&mut world, &humanoid(), &bind, &pose, None);
    for (bone, p) in ragdoll.bone_poses(&world) {
        assert!((p.translation - pose[bone].translation).length() < 1e-6, "bone {bone}");
    }
}

#[test]
fn falls_and_comes_to_rest_on_floor() {
    let mut world = world_with_floor();
    let bind = bind_pose();
    let pose = lifted(&bind, Vector::new(0.0, 1.0, 0.0));
    let ragdoll = Ragdoll::spawn(&mut world, &humanoid(), &bind, &pose, None);
    run(&mut world, 4.0);

    let defs = humanoid();
    for def in &defs {
        let p = pose_of(&ragdoll, &world, def.bone);
        let lowest = p.translation.y.min((p * def.end).y) - def.radius;
        assert!(lowest > -0.03, "bone {} sank to {lowest}", def.bone);
        let speed = ragdoll.linear_velocity(&world, def.bone).unwrap().length();
        assert!(speed < 0.2, "bone {} still moving at {speed}", def.bone);
    }
    let pelvis = pose_of(&ragdoll, &world, PELVIS).translation.y;
    assert!(pelvis < 0.5, "pelvis at {pelvis}: body did not collapse");
}

#[test]
fn joints_stay_connected_after_fall() {
    let mut world = world_with_floor();
    let bind = bind_pose();
    let pose = lifted(&bind, Vector::new(0.0, 2.0, 0.0));
    let ragdoll = Ragdoll::spawn(&mut world, &humanoid(), &bind, &pose, None);
    run(&mut world, 3.0);

    let defs = humanoid();
    for def in &defs {
        let Some(parent) = def.parent else { continue };
        let parent_bone = defs[parent].bone;
        let anchor =
            pose_of(&ragdoll, &world, parent_bone) * (bind[parent_bone].inverse() * bind[def.bone]).translation;
        let gap = (anchor - pose_of(&ragdoll, &world, def.bone).translation).length();
        assert!(gap < 0.02, "joint {parent_bone}->{} separated by {gap}", def.bone);
    }
}

#[test]
fn hinge_limits_hold() {
    let mut world = world_with_floor();
    let bind = bind_pose();
    // Throw the body forward so knees and elbows get loaded.
    let pose = lifted(&bind, Vector::new(0.0, 1.5, 0.0));
    let previous = lifted(&bind, Vector::new(0.0, 1.5, -0.1));
    let ragdoll = Ragdoll::spawn(&mut world, &humanoid(), &bind, &pose, Some((&previous, 0.05)));
    let tolerance = 0.15;
    for _ in 0..180 {
        world.advance(1.0 / 60.0);
        for (parent, child, hinge) in [
            (L_THIGH, L_SHIN, KNEE),
            (R_THIGH, R_SHIN, KNEE),
            (L_UPPER_ARM, L_FOREARM, L_ELBOW),
            (R_UPPER_ARM, R_FOREARM, R_ELBOW),
        ] {
            let JointDef::Hinge {
                axis: hinge_axis,
                limits,
            } = hinge
            else {
                unreachable!()
            };
            let (axis, angle) = joint_rotation(&ragdoll, &world, &bind, parent, child).to_axis_angle();
            let along = axis.dot(hinge_axis);
            assert!(
                along.abs() > 0.9 || angle < tolerance,
                "hinge {child} bends off-axis: {axis:?} {angle}"
            );
            let signed = if along >= 0.0 { angle } else { -angle };
            assert!(
                signed > limits[0] - tolerance && signed < limits[1] + tolerance,
                "hinge {child} at {signed}, limits {limits:?}"
            );
        }
    }
}

#[test]
fn impulse_pushes_hit_bone_hardest() {
    let mut world = RagdollWorld::new();
    world.physics.gravity = Vector::ZERO;
    let bind = bind_pose();
    let ragdoll = Ragdoll::spawn(&mut world, &humanoid(), &bind, &bind, None);
    let head = bind[HEAD].translation + Vector::new(0.0, 0.1, 0.0);
    assert!(ragdoll.apply_impulse(&mut world, HEAD, head, Vector::new(0.0, 0.0, 8.0)));
    assert!(!ragdoll.apply_impulse(&mut world, 99, head, Vector::Z), "unknown bone");
    run(&mut world, 0.1);
    let head_dz = pose_of(&ragdoll, &world, HEAD).translation.z;
    let foot_dz = pose_of(&ragdoll, &world, L_SHIN).translation.z;
    // The joints spread the push over the whole body (about 70 kg).
    assert!(head_dz > 0.01, "head moved {head_dz}");
    assert!(head_dz > foot_dz.abs() * 2.0, "head {head_dz} vs shin {foot_dz}");
}

#[test]
fn inherits_animation_velocity() {
    let mut world = RagdollWorld::new();
    world.physics.gravity = Vector::ZERO;
    let bind = bind_pose();
    let previous = lifted(&bind, Vector::new(-0.1, 0.0, 0.0));
    let ragdoll = Ragdoll::spawn(&mut world, &humanoid(), &bind, &bind, Some((&previous, 0.1)));
    let v = ragdoll.linear_velocity(&world, PELVIS).unwrap();
    assert!(
        (v - Vector::new(1.0, 0.0, 0.0)).length() < 1e-4,
        "pelvis velocity {v:?}"
    );
    run(&mut world, 0.5);
    let x = pose_of(&ragdoll, &world, PELVIS).translation.x;
    assert!((x - 0.5).abs() < 0.05, "pelvis drifted to {x}");
}

#[test]
fn result_does_not_depend_on_frame_rate() {
    let simulate = |fps: u32| {
        let mut world = world_with_floor();
        let bind = bind_pose();
        let pose = lifted(&bind, Vector::new(0.0, 1.0, 0.0));
        let ragdoll = Ragdoll::spawn(&mut world, &humanoid(), &bind, &pose, None);
        let mut steps = 0;
        for _ in 0..fps {
            steps += world.advance(1.0 / fps as f32);
        }
        (steps, pose_of(&ragdoll, &world, HEAD).translation)
    };
    let (steps_30, head_30) = simulate(30);
    let (steps_144, head_144) = simulate(144);
    assert_eq!(steps_30, 120);
    assert_eq!(steps_144, 120);
    assert!((head_30 - head_144).length() < 1e-4, "{head_30:?} vs {head_144:?}");
}

#[test]
fn bullet_time_slows_the_fall() {
    let fall = |time_scale: f32| {
        let mut world = RagdollWorld::new();
        let bind = bind_pose();
        let ragdoll = Ragdoll::spawn(&mut world, &humanoid(), &bind, &bind, None);
        for _ in 0..60 {
            world.advance(time_scale / 60.0);
        }
        bind[PELVIS].translation.y - pose_of(&ragdoll, &world, PELVIS).translation.y
    };
    let normal = fall(1.0);
    let slow = fall(0.25);
    // Free fall distance scales with t^2: a quarter of the time gives a sixteenth.
    assert!((normal - 4.905).abs() < 0.1, "normal fall {normal}");
    assert!((slow * 16.0 - normal).abs() < 0.15, "slow fall {slow}");
}

#[test]
fn long_hitch_is_capped() {
    let mut world = RagdollWorld::new();
    assert_eq!(world.advance(5.0), ragdoll::MAX_STEPS_PER_ADVANCE);
    assert!(world.advance(0.0) <= 1, "backlog was dropped");
    assert_eq!(world.advance(FIXED_DT), 1);
}
