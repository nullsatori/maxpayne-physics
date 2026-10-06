use ragdoll::{BodyDef, FIXED_DT, JointDef, MAX_TRACK_SPEED, RagdollId, RagdollWorld};
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

fn pose_of(world: &RagdollWorld, ragdoll: RagdollId, bone: usize) -> Pose {
    world
        .bone_poses(ragdoll)
        .into_iter()
        .find(|(b, _)| *b == bone)
        .unwrap()
        .1
}

/// Rotation of `child` relative to `parent`, measured from the bind pose.
fn joint_rotation(world: &RagdollWorld, ragdoll: RagdollId, bind: &[Pose], parent: usize, child: usize) -> Rotation {
    let rel_now = pose_of(world, ragdoll, parent).inverse() * pose_of(world, ragdoll, child);
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
    let ragdoll = world.spawn(&humanoid(), &bind, &pose, None);
    for (bone, p) in world.bone_poses(ragdoll) {
        assert!((p.translation - pose[bone].translation).length() < 1e-6, "bone {bone}");
    }
}

#[test]
fn falls_and_comes_to_rest_on_floor() {
    let mut world = world_with_floor();
    let bind = bind_pose();
    let pose = lifted(&bind, Vector::new(0.0, 1.0, 0.0));
    let ragdoll = world.spawn(&humanoid(), &bind, &pose, None);
    run(&mut world, 4.0);

    let defs = humanoid();
    for def in &defs {
        let p = pose_of(&world, ragdoll, def.bone);
        let lowest = p.translation.y.min((p * def.end).y) - def.radius;
        assert!(lowest > -0.03, "bone {} sank to {lowest}", def.bone);
        let speed = world.linear_velocity(ragdoll, def.bone).unwrap().length();
        assert!(speed < 0.2, "bone {} still moving at {speed}", def.bone);
    }
    let pelvis = pose_of(&world, ragdoll, PELVIS).translation.y;
    assert!(pelvis < 0.5, "pelvis at {pelvis}: body did not collapse");
}

#[test]
fn joints_stay_connected_after_fall() {
    let mut world = world_with_floor();
    let bind = bind_pose();
    let pose = lifted(&bind, Vector::new(0.0, 2.0, 0.0));
    let ragdoll = world.spawn(&humanoid(), &bind, &pose, None);
    run(&mut world, 3.0);

    let defs = humanoid();
    for def in &defs {
        let Some(parent) = def.parent else { continue };
        let parent_bone = defs[parent].bone;
        let anchor = pose_of(&world, ragdoll, parent_bone) * (bind[parent_bone].inverse() * bind[def.bone]).translation;
        let gap = (anchor - pose_of(&world, ragdoll, def.bone).translation).length();
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
    let ragdoll = world.spawn(&humanoid(), &bind, &pose, Some((&previous, 0.05)));
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
            let (axis, angle) = joint_rotation(&world, ragdoll, &bind, parent, child).to_axis_angle();
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
    let ragdoll = world.spawn(&humanoid(), &bind, &bind, None);
    let head = bind[HEAD].translation + Vector::new(0.0, 0.1, 0.0);
    assert!(world.apply_impulse(ragdoll, HEAD, head, Vector::new(0.0, 0.0, 8.0)));
    assert!(!world.apply_impulse(ragdoll, 99, head, Vector::Z), "unknown bone");
    run(&mut world, 0.1);
    let head_dz = pose_of(&world, ragdoll, HEAD).translation.z;
    let foot_dz = pose_of(&world, ragdoll, L_SHIN).translation.z;
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
    let ragdoll = world.spawn(&humanoid(), &bind, &bind, Some((&previous, 0.1)));
    let v = world.linear_velocity(ragdoll, PELVIS).unwrap();
    assert!(
        (v - Vector::new(1.0, 0.0, 0.0)).length() < 1e-4,
        "pelvis velocity {v:?}"
    );
    run(&mut world, 0.5);
    let x = pose_of(&world, ragdoll, PELVIS).translation.x;
    assert!((x - 0.5).abs() < 0.05, "pelvis drifted to {x}");
}

#[test]
fn result_does_not_depend_on_frame_rate() {
    let simulate = |fps: u32| {
        let mut world = world_with_floor();
        let bind = bind_pose();
        let pose = lifted(&bind, Vector::new(0.0, 1.0, 0.0));
        let ragdoll = world.spawn(&humanoid(), &bind, &pose, None);
        let mut steps = 0;
        for _ in 0..fps {
            steps += world.advance(1.0 / fps as f32);
        }
        (steps, pose_of(&world, ragdoll, HEAD).translation)
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
        let ragdoll = world.spawn(&humanoid(), &bind, &bind, None);
        for _ in 0..60 {
            world.advance(time_scale / 60.0);
        }
        bind[PELVIS].translation.y - pose_of(&world, ragdoll, PELVIS).translation.y
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

#[test]
fn despawn_removes_bodies() {
    let mut world = RagdollWorld::new();
    let bind = bind_pose();
    let first = world.spawn(&humanoid(), &bind, &bind, None);
    let second = world.spawn(&humanoid(), &bind, &bind, None);
    assert_ne!(first, second);
    world.despawn(first);
    assert_eq!(world.physics.bodies.len(), humanoid().len());
    assert_eq!(world.physics.impulse_joints.len(), humanoid().len() - 1);
    assert_eq!(world.bone_poses(second).len(), humanoid().len());
    let third = world.spawn(&humanoid(), &bind, &bind, None);
    assert_ne!(third, first, "ids are not reused");
}

// Pose tracking.

/// Bind pose carried by `root`, with elbows and knees bent by the given
/// angles toward the side their hinge limits allow.
fn posed(root: Pose, elbow: f32, knee: f32) -> Vec<Pose> {
    let bind = bind_pose();
    let defs = humanoid();
    let mut pose = bind.clone();
    for def in &defs {
        pose[def.bone] = match def.parent {
            None => root * bind[def.bone],
            Some(parent) => {
                let bend = match def.joint {
                    JointDef::Hinge { axis, limits } => {
                        let angle = if def.bone == L_SHIN || def.bone == R_SHIN {
                            knee
                        } else {
                            elbow
                        };
                        Rotation::from_axis_angle(axis, if limits[0] < 0.0 { -angle } else { angle })
                    }
                    JointDef::Ball { .. } => Rotation::IDENTITY,
                };
                let parent_bone = defs[parent].bone;
                pose[parent_bone] * bind[parent_bone].inverse() * bind[def.bone] * Pose::from_rotation(bend)
            }
        };
    }
    pose
}

/// Test animation at game time `t`: the body circles at 0.75 m/s while it
/// turns, bobs and flexes elbows and knees.
fn dance(t: f32) -> Vec<Pose> {
    let center = Vector::new(
        0.5 * (1.5 * t).sin(),
        0.2 * (2.0 * t).sin(),
        0.5 * (1.5 * t).cos() - 0.5,
    );
    let root = Pose::from_parts(center, Rotation::from_rotation_y(0.8 * t));
    posed(root, 1.2 + 0.8 * (3.0 * t).sin(), 0.6 + 0.5 * (3.0 * t + 1.0).sin())
}

/// Angle between two rotations, precise near zero unlike `angle_between`.
fn angle(a: Rotation, b: Rotation) -> f32 {
    2.0 * (a.inverse() * b).xyz().length().min(1.0).asin()
}

/// Largest distance and angle between matching bones of two pose lists.
fn difference(a: &[(usize, Pose)], b: &[(usize, Pose)]) -> (f32, f32) {
    a.iter()
        .zip(b)
        .fold((0.0, 0.0), |(dist, turn), ((bone_a, p), (bone_b, q))| {
            assert_eq!(bone_a, bone_b);
            let d = (p.translation - q.translation).length();
            (dist.max(d), turn.max(angle(p.rotation, q.rotation)))
        })
}

/// Largest distance between a joint anchor on the parent body and the child bone.
fn joint_gap(world: &RagdollWorld, ragdoll: RagdollId) -> f32 {
    let defs = humanoid();
    let bind = bind_pose();
    let mut gap = 0.0f32;
    for def in &defs {
        let Some(parent) = def.parent else { continue };
        let parent_bone = defs[parent].bone;
        let anchor = pose_of(world, ragdoll, parent_bone) * (bind[parent_bone].inverse() * bind[def.bone]).translation;
        gap = gap.max((anchor - pose_of(world, ragdoll, def.bone).translation).length());
    }
    gap
}

fn tracking_error(world: &RagdollWorld, ragdoll: RagdollId, target: &[Pose]) -> (f32, f32) {
    let poses = world.bone_poses(ragdoll);
    let targets: Vec<_> = poses.iter().map(|(bone, _)| (*bone, target[*bone])).collect();
    difference(&poses, &targets)
}

fn zero_gravity() -> RagdollWorld {
    let mut world = RagdollWorld::new();
    world.physics.gravity = Vector::ZERO;
    world
}

/// Spawns a ragdoll at `animation(0)` and tracks the animation at full
/// strength for `seconds` of game time, rendering `fps` frames per second of
/// real time with game time running at `time_scale`. `frame` runs after
/// every frame with the game time reached.
///
/// Tracking starts at spawn, so the first frame already moves from the
/// spawn pose toward the next one.
fn track_animation(
    world: &mut RagdollWorld,
    animation: impl Fn(f32) -> Vec<Pose>,
    fps: u32,
    time_scale: f32,
    seconds: f32,
    mut frame: impl FnMut(&mut RagdollWorld, RagdollId, f32),
) -> RagdollId {
    let start = animation(0.0);
    let ragdoll = world.spawn(&humanoid(), &bind_pose(), &start, None);
    world.track(ragdoll, &start, 1.0);
    let dt = time_scale / fps as f32;
    let frames = (seconds / dt).round() as u32;
    for i in 1..=frames {
        let t = i as f32 * dt;
        world.track(ragdoll, &animation(t), 1.0);
        world.advance(dt);
        frame(world, ragdoll, t);
    }
    ragdoll
}

#[test]
fn tracks_moving_target() {
    let mut world = zero_gravity();
    let (mut dist, mut turn, mut gap) = (0.0f32, 0.0f32, 0.0f32);
    let ragdoll = track_animation(&mut world, dance, 60, 1.0, 3.0, |world, ragdoll, t| {
        // Motion starts from rest; the body catches up with its velocity first.
        if t < 0.5 {
            return;
        }
        let (d, a) = tracking_error(world, ragdoll, &dance(t));
        dist = dist.max(d);
        turn = turn.max(a);
        gap = gap.max(joint_gap(world, ragdoll));
    });
    // Tolerance: 5 mm and 0.01 rad on every bone, against a path that
    // carries the body 0.8 m and turns it 2.4 rad.
    assert!(dist < 0.005, "bone off its target by {dist} m");
    assert!(turn < 0.01, "bone off its target by {turn} rad");
    assert!(gap < 0.005, "joint separated by {gap}");
    let (moved, turned) = tracking_error(&world, ragdoll, &dance(0.0));
    assert!(moved > 0.7 && turned > 2.0, "animation moved {moved} m, {turned} rad");
}

#[test]
fn tracking_does_not_depend_on_frame_rate() {
    // Poses every half second of game time.
    let simulate = |fps: u32| {
        let mut world = zero_gravity();
        let mut samples = Vec::new();
        let mut frames = 0;
        track_animation(&mut world, dance, fps, 1.0, 2.0, |world, ragdoll, _| {
            frames += 1;
            if frames % (fps / 2) == 0 {
                samples.push(world.bone_poses(ragdoll));
            }
        });
        samples
    };
    let reference = simulate(144);
    assert_eq!(reference.len(), 4);
    for fps in [30, 60] {
        for (poses, expected) in simulate(fps).iter().zip(&reference) {
            let (dist, turn) = difference(poses, expected);
            assert!(dist < 0.002 && turn < 0.005, "{fps} fps: off by {dist} m, {turn} rad");
        }
    }
}

#[test]
fn wall_holds_back_tracked_body() {
    // The target jumps 2.5 m behind a wall without thickness, so the drive
    // runs into the wall at its limits and keeps pushing. Standing on a floor
    // the body hits the wall early; in zero gravity it reaches top speed first.
    for (mut world, wall) in [(world_with_floor(), 1.5), (zero_gravity(), 8.0)] {
        let vertices = vec![
            Vector::new(-3.0, -3.0, wall),
            Vector::new(3.0, -3.0, wall),
            Vector::new(3.0, 3.0, wall),
            Vector::new(-3.0, 3.0, wall),
        ];
        world.add_static_mesh(vertices, vec![[0, 1, 2], [0, 2, 3]]).unwrap();
        let defs = humanoid();
        let bind = bind_pose();
        let jump = |t: f32| lifted(&bind, Vector::new(0.0, 0.0, if t > 0.0 { wall + 2.5 } else { 0.0 }));
        let (mut depth, mut late_depth, mut speed, mut late_speed) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
        let ragdoll = track_animation(&mut world, jump, 60, 1.0, 3.0, |world, ragdoll, t| {
            for def in &defs {
                let p = pose_of(world, ragdoll, def.bone);
                let d = p.translation.z.max((p * def.end).z) + def.radius - wall;
                let v = world.linear_velocity(ragdoll, def.bone).unwrap().length();
                depth = depth.max(d);
                speed = speed.max(v);
                if t > 2.0 {
                    late_depth = late_depth.max(d);
                    late_speed = late_speed.max(v);
                }
            }
        });
        let case = format!("wall at {wall}");
        assert!(depth < 0.01, "{case}: penetrated by {depth}");
        assert!(late_depth < 0.002, "{case}: pressed in by {late_depth}");
        // Limbs whip a little past the drive's top speed when the trunk stops.
        assert!(speed < 1.25 * MAX_TRACK_SPEED, "{case}: reached {speed} m/s");
        assert!(late_speed < 0.3, "{case}: still moving at {late_speed} m/s");
        let chest = pose_of(&world, ragdoll, SPINE).translation.z + 0.14;
        assert!(wall - chest < 0.01, "{case}: chest at {chest} did not reach the wall");
    }
}

#[test]
fn zero_strength_is_passive() {
    let fall = |strength: Option<f32>| {
        let mut world = world_with_floor();
        let bind = bind_pose();
        let pose = lifted(&bind, Vector::new(0.0, 1.0, 0.0));
        let ragdoll = world.spawn(&humanoid(), &bind, &pose, None);
        for _ in 0..120 {
            if let Some(strength) = strength {
                world.track(ragdoll, &pose, strength);
            }
            world.advance(1.0 / 60.0);
        }
        world.bone_poses(ragdoll)
    };
    let passive = fall(None);
    let (dist, turn) = difference(&fall(Some(0.0)), &passive);
    assert!(dist == 0.0 && turn == 0.0, "strength 0 differs by {dist} m, {turn} rad");
    // The same target at a little strength already changes the outcome.
    let (dist, _) = difference(&fall(Some(0.3)), &passive);
    assert!(dist > 0.3, "strength 0.3 differs by only {dist}");
}

#[test]
fn strength_blends_between_passive_and_tracked() {
    // A body held in the air: weaker drives sag more under gravity.
    let drops: Vec<f32> = [0.0, 0.1, 0.25, 0.5, 0.75, 1.0]
        .into_iter()
        .map(|strength| {
            let mut world = RagdollWorld::new();
            let bind = bind_pose();
            let ragdoll = world.spawn(&humanoid(), &bind, &bind, None);
            for _ in 0..60 {
                world.track(ragdoll, &bind, strength);
                world.advance(1.0 / 60.0);
            }
            bind[PELVIS].translation.y - pose_of(&world, ragdoll, PELVIS).translation.y
        })
        .collect();
    assert!(drops[0] > 4.5, "passive body fell only {}", drops[0]);
    assert!(drops[5] < 0.01, "full strength sagged {}", drops[5]);
    assert!(
        drops.windows(2).all(|w| w[0] > w[1]),
        "drop is not monotonic: {drops:?}"
    );
}

#[test]
fn bullet_knocks_tracked_body_off_and_it_recovers() {
    let mut world = world_with_floor();
    let bind = bind_pose();
    let stand = |_: f32| bind.clone();
    let mut hit = None;
    let (mut before, mut peak, mut after) = (0.0f32, 0.0f32, 0.0f32);
    track_animation(&mut world, stand, 60, 1.0, 2.0, |world, ragdoll, t| {
        let (dev, _) = tracking_error(world, ragdoll, &bind);
        let Some(hit_at) = hit else {
            before = before.max(dev);
            if t >= 0.5 {
                // A shot from the front into the right side of the chest, several
                // times a real bullet's momentum as games exaggerate hits.
                let point = bind[SPINE].translation + Vector::new(-0.1, 0.25, 0.14);
                assert!(world.apply_impulse(ragdoll, SPINE, point, Vector::new(0.0, 0.0, -60.0)));
                hit = Some(t);
            }
            return;
        };
        if t - hit_at < 0.5 {
            peak = peak.max(dev);
        } else {
            after = after.max(dev);
        }
    });
    assert!(peak > 0.05, "hit moved the body only {peak} m");
    assert!(
        after < 0.01 && after < before * 2.0,
        "did not recover: {after} m off, {before} m before the hit"
    );
}

#[test]
fn bullet_time_slows_tracking() {
    let path = |time_scale: f32| {
        let mut world = zero_gravity();
        let mut path = Vec::new();
        track_animation(&mut world, dance, 60, time_scale, 1.0, |world, ragdoll, _| {
            path.push(world.bone_poses(ragdoll));
        });
        path
    };
    let normal = path(1.0);
    let slow = path(0.25);
    assert_eq!(slow.len(), 4 * normal.len());
    // Four bullet time frames span the game time of one normal frame.
    for (i, poses) in normal.iter().enumerate() {
        let (dist, turn) = difference(&slow[4 * i + 3], poses);
        assert!(dist < 0.002 && turn < 0.005, "frame {i}: off by {dist} m, {turn} rad");
    }
    let (moved, _) = difference(&normal[59], &normal[0]);
    let (moved_slow, _) = difference(&slow[59], &slow[0]);
    assert!(
        moved_slow < moved * 0.4,
        "bullet time moved {moved_slow} m in the frames normal time moved {moved}"
    );
}

#[test]
fn quaternion_sign_does_not_matter() {
    let flip = |pose: Vec<Pose>| -> Vec<Pose> {
        pose.into_iter()
            .map(|p| Pose::from_parts(p.translation, -p.rotation))
            .collect()
    };
    let flipped_every_other = |t: f32| {
        let pose = dance(t);
        if (t * 60.0).round() as u32 % 2 == 1 {
            flip(pose)
        } else {
            pose
        }
    };
    let simulate = |animation: &dyn Fn(f32) -> Vec<Pose>| {
        let mut world = zero_gravity();
        let ragdoll = track_animation(&mut world, animation, 60, 1.0, 1.0, |_, _, _| {});
        world.bone_poses(ragdoll)
    };
    let (dist, turn) = difference(&simulate(&flipped_every_other), &simulate(&dance));
    assert!(dist < 1e-4 && turn < 1e-3, "tracking: off by {dist} m, {turn} rad");

    // Spawn velocity from a previous pose stored with opposite signs.
    let spin = |previous: Vec<Pose>| {
        let mut world = zero_gravity();
        let ragdoll = world.spawn(&humanoid(), &bind_pose(), &dance(0.1), Some((&previous, 0.1)));
        run(&mut world, 0.5);
        world.bone_poses(ragdoll)
    };
    let (dist, turn) = difference(&spin(flip(dance(0.0))), &spin(dance(0.0)));
    assert!(dist < 1e-4 && turn < 1e-3, "spawn: off by {dist} m, {turn} rad");
}

#[test]
fn joint_limits_beat_tracking() {
    let mut world = world_with_floor();
    let defs = humanoid();
    let bind = bind_pose();
    // Elbows and knees asked to bend 1 rad the wrong way, past their limit
    // at 0, as a retargeted pose may do.
    let wrong = |t: f32| {
        let bend = if t > 0.0 { -1.0 } else { 0.0 };
        posed(Pose::IDENTITY, bend, bend)
    };
    let (mut bent, mut gap, mut late_speed) = (0.0f32, 0.0f32, 0.0f32);
    track_animation(&mut world, wrong, 60, 1.0, 2.0, |world, ragdoll, t| {
        for (parent, child) in [
            (L_THIGH, L_SHIN),
            (R_THIGH, R_SHIN),
            (L_UPPER_ARM, L_FOREARM),
            (R_UPPER_ARM, R_FOREARM),
        ] {
            bent = bent.max(angle(
                joint_rotation(world, ragdoll, &bind, parent, child),
                Rotation::IDENTITY,
            ));
        }
        gap = gap.max(joint_gap(world, ragdoll));
        if t > 1.0 {
            for def in &defs {
                late_speed = late_speed.max(world.linear_velocity(ragdoll, def.bone).unwrap().length());
            }
        }
    });
    assert!(bent < 0.05, "hinge bent {bent} rad past its limit");
    assert!(gap < 0.005, "joint separated by {gap}");
    assert!(late_speed < 0.2, "still moving at {late_speed} m/s");
}
