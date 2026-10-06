//! Game-independent ragdoll simulation.
//!
//! Units are meters and seconds with +Y up; the game adapter converts game
//! units and axes before calling in. Each ragdoll body is attached to one
//! skeleton bone: the rigid body pose *is* the bone's world pose, so reading
//! a pose back needs no retargeting offsets.
//!
//! A ragdoll can track an animated pose (in MP1, the pose streamed from MP3's
//! animation and Euphoria). Every fixed step its bodies are steered toward the
//! targets by velocity changes with bounded speed and acceleration, so level
//! geometry, joints and bullet impulses still win, and the body recovers after.

use std::collections::BTreeMap;

use rapier3d::prelude::*;

/// Fixed simulation step. Game frames are split into steps of this size so
/// the result does not depend on frame rate or bullet time scale.
pub const FIXED_DT: Real = 1.0 / 120.0;

/// Upper bound on fixed steps per `advance` call, so a long hitch does not
/// stall the game; the remaining time is dropped.
pub const MAX_STEPS_PER_ADVANCE: u32 = 16;

/// Time constant at full tracking strength with which a pose error closes.
pub const TRACK_TIME: Real = 0.05;

/// Speed the tracking drive may ask for. Bounds the chase after a target that
/// jumped or sits behind a wall; hits and falls can still make bodies faster.
pub const MAX_TRACK_SPEED: Real = 20.0;

/// Angular counterpart of [`MAX_TRACK_SPEED`], radians per second.
pub const MAX_TRACK_ANGULAR_SPEED: Real = 30.0;

/// Acceleration (force per kilogram) of the tracking drive at full strength.
/// About 3 g: enough to hold a body up and keep up with animation, low enough
/// that a hit visibly knocks the body off its target before it comes back.
pub const MAX_TRACK_ACCEL: Real = 30.0;

/// Angular counterpart of [`MAX_TRACK_ACCEL`], radians per second squared.
pub const MAX_TRACK_ANGULAR_ACCEL: Real = 300.0;

/// Rotation limits of a joint, in radians, around the child bone's bind
/// frame axes.
#[derive(Clone, Copy, Debug)]
pub enum JointDef {
    /// Free rotation inside per-axis limits (hips, shoulders, neck).
    Ball { x: [Real; 2], y: [Real; 2], z: [Real; 2] },
    /// Rotation around one axis of the child bone's bind frame (knees, elbows).
    Hinge { axis: Vector, limits: [Real; 2] },
}

/// One rigid body of a ragdoll, attached to a skeleton bone.
#[derive(Clone, Copy, Debug)]
pub struct BodyDef {
    /// Bone index into the pose arrays passed to [`RagdollWorld::spawn`].
    pub bone: usize,
    /// Index of the parent entry in the same `BodyDef` slice; `None` for the root.
    pub parent: Option<usize>,
    /// Capsule end point in the bone's local frame; the capsule starts at the bone origin.
    pub end: Vector,
    pub radius: Real,
    /// kg/m^3; the human body averages roughly 1000.
    pub density: Real,
    /// Joint to the parent body; ignored for the root.
    pub joint: JointDef,
}

/// A ragdoll in a [`RagdollWorld`]. Ids are not reused: the id of a despawned
/// ragdoll panics instead of reaching another ragdoll.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RagdollId(u64);

/// The physics world and the ragdolls in it. The world owns the ragdolls
/// because tracking acts on them before every fixed step.
pub struct RagdollWorld {
    pub physics: PhysicsWorld,
    ragdolls: BTreeMap<RagdollId, Ragdoll>,
    next_id: u64,
    accumulator: Real,
}

struct Ragdoll {
    bones: Vec<usize>,
    bodies: Vec<RigidBodyHandle>,
    tracking: Option<Tracking>,
}

/// Targets per body (in `Ragdoll::bodies` order) at the start and the end of
/// the next `advance`; the steps in between follow the interpolated pose.
struct Tracking {
    start: Vec<Pose>,
    end: Vec<Pose>,
    strength: Real,
}

impl Default for RagdollWorld {
    fn default() -> Self {
        Self::new()
    }
}

impl RagdollWorld {
    pub fn new() -> Self {
        let mut physics = PhysicsWorld::new();
        physics.integration_parameters.dt = FIXED_DT;
        Self {
            physics,
            ragdolls: BTreeMap::new(),
            next_id: 0,
            accumulator: 0.0,
        }
    }

    /// Adds static level geometry, e.g. collision triangles of a level.
    pub fn add_static_mesh(
        &mut self,
        vertices: Vec<Vector>,
        triangles: Vec<[u32; 3]>,
    ) -> Result<ColliderHandle, TriMeshBuilderError> {
        let collider = ColliderBuilder::trimesh(vertices, triangles)?.friction(0.8);
        Ok(self.physics.insert_collider(collider, None))
    }

    /// Creates a passive ragdoll with bodies matching the current animated pose.
    ///
    /// `bind` holds bind-pose bone transforms in model space and defines the
    /// neutral joint orientation that limits are measured from. `pose` holds
    /// current world transforms. `previous`, when given, is the world pose
    /// `dt` seconds earlier and lets the ragdoll inherit animation momentum.
    pub fn spawn(
        &mut self,
        defs: &[BodyDef],
        bind: &[Pose],
        pose: &[Pose],
        previous: Option<(&[Pose], Real)>,
    ) -> RagdollId {
        let mut bodies = Vec::with_capacity(defs.len());
        for (index, def) in defs.iter().enumerate() {
            let current = pose[def.bone];
            let mut builder = RigidBodyBuilder::dynamic()
                .pose(current)
                .ccd_enabled(true)
                .linear_damping(0.05)
                .angular_damping(0.3);
            if let Some((prev, dt)) = previous.filter(|(_, dt)| *dt > 0.0) {
                let before = prev[def.bone];
                builder = builder
                    .linvel((current.translation - before.translation) / dt)
                    .angvel(rotation_vector(current.rotation * before.rotation.inverse()) / dt);
            }
            let handle = self.physics.insert_body(builder);
            let collider = ColliderBuilder::capsule_from_endpoints(Vector::ZERO, def.end, def.radius)
                .density(def.density)
                .friction(0.7);
            self.physics.insert_collider(collider, Some(handle));

            if let Some(parent) = def.parent {
                assert!(parent < index, "BodyDef parent must precede its child");
                let parent_bone = defs[parent].bone;
                // Joint frame: child bone origin, axes of the child bind frame
                // (rotated so X is the hinge axis), expressed in both bodies.
                let (mask, axis_frame) = match def.joint {
                    JointDef::Ball { .. } => (JointAxesMask::LOCKED_SPHERICAL_AXES, Pose::IDENTITY),
                    JointDef::Hinge { axis, .. } => (
                        JointAxesMask::LOCKED_REVOLUTE_AXES,
                        Pose::from_rotation(Rotation::from_rotation_arc(Vector::X, axis.normalize())),
                    ),
                };
                let mut joint = GenericJointBuilder::new(mask)
                    .local_frame1(bind[parent_bone].inverse() * bind[def.bone] * axis_frame)
                    .local_frame2(axis_frame)
                    .contacts_enabled(false);
                joint = match def.joint {
                    JointDef::Ball { x, y, z } => joint
                        .limits(JointAxis::AngX, x)
                        .limits(JointAxis::AngY, y)
                        .limits(JointAxis::AngZ, z),
                    JointDef::Hinge { limits, .. } => joint.limits(JointAxis::AngX, limits),
                };
                let joint = joint.build();
                self.physics.insert_impulse_joint(bodies[parent], handle, joint);
            }
            bodies.push(handle);
        }
        let id = RagdollId(self.next_id);
        self.next_id += 1;
        let ragdoll = Ragdoll {
            bones: defs.iter().map(|d| d.bone).collect(),
            bodies,
            tracking: None,
        };
        self.ragdolls.insert(id, ragdoll);
        id
    }

    /// Removes the ragdoll's bodies and joints.
    pub fn despawn(&mut self, id: RagdollId) {
        let ragdoll = self.ragdolls.remove(&id).expect("unknown RagdollId");
        for handle in ragdoll.bodies {
            self.physics.remove_body(handle);
        }
    }

    /// Makes the ragdoll follow an animated pose.
    ///
    /// `pose` holds world bone transforms indexed like in [`Self::spawn`] and
    /// is where the animation will be at the end of the next
    /// [`Self::advance`]. Call it every frame before `advance`: the fixed steps
    /// of a frame follow the pose interpolated from the previous target, so
    /// the result does not depend on frame rate. A frame without a call holds
    /// the last target. So does the first frame of tracking, having nothing to
    /// interpolate from; passing the spawn pose right after `spawn` gives it one.
    ///
    /// `strength` in [0, 1] scales the drive's stiffness, damping and
    /// acceleration limit together: response speed grows with strength at a
    /// constant damping ratio, so a weak drive lags and sags under gravity like
    /// weak muscles instead of turning springy. 0 drops the targets and leaves
    /// a passive ragdoll.
    pub fn track(&mut self, id: RagdollId, pose: &[Pose], strength: Real) {
        let ragdoll = self.ragdolls.get_mut(&id).expect("unknown RagdollId");
        // Compared this way round, a NaN strength stops tracking as well.
        let Some(strength) = (strength > 0.0).then_some(strength.min(1.0)) else {
            ragdoll.tracking = None;
            return;
        };
        let targets = ragdoll.bones.iter().map(|bone| pose[*bone]);
        match &mut ragdoll.tracking {
            Some(tracking) => {
                tracking.end.clear();
                tracking.end.extend(targets);
                tracking.strength = strength;
            }
            None => {
                // Nothing to interpolate from yet: hold the first target.
                let end: Vec<Pose> = targets.collect();
                ragdoll.tracking = Some(Tracking {
                    start: end.clone(),
                    end,
                    strength,
                });
            }
        }
    }

    /// Current world pose of every simulated bone, as `(bone index, pose)`.
    pub fn bone_poses(&self, id: RagdollId) -> Vec<(usize, Pose)> {
        let ragdoll = self.ragdoll(id);
        ragdoll
            .bones
            .iter()
            .zip(&ragdoll.bodies)
            .map(|(bone, handle)| (*bone, *self.physics.bodies[*handle].position()))
            .collect()
    }

    /// Applies a bullet hit at a world point to the body of `bone`.
    /// Returns false when the bone is not part of the ragdoll.
    pub fn apply_impulse(&mut self, id: RagdollId, bone: usize, point: Vector, impulse: Vector) -> bool {
        let Some(handle) = self.ragdoll(id).body(bone) else {
            return false;
        };
        self.physics.bodies[handle].apply_impulse_at_point(impulse, point, true);
        true
    }

    pub fn linear_velocity(&self, id: RagdollId, bone: usize) -> Option<Vector> {
        let handle = self.ragdoll(id).body(bone)?;
        Some(self.physics.bodies[handle].linvel())
    }

    /// Advances by `dt` seconds of game time. Bullet time is expressed by the
    /// game passing scaled time, which keeps every step at `FIXED_DT`.
    /// Tracking ragdolls are steered toward their targets before every step.
    /// Returns the number of fixed steps taken.
    pub fn advance(&mut self, dt: Real) -> u32 {
        // Tolerance absorbs float error when frames sum to whole steps.
        const EPS: Real = 1.0e-5;
        let dt = dt.max(0.0);
        // Physics lags game time by the accumulator; the end of each step is
        // placed in game time from the start of this frame to pick its target.
        let lag = self.accumulator;
        self.accumulator += dt;
        let mut steps = 0;
        while self.accumulator + EPS >= FIXED_DT && steps < MAX_STEPS_PER_ADVANCE {
            steps += 1;
            let elapsed = steps as Real * FIXED_DT - lag;
            for ragdoll in self.ragdolls.values() {
                ragdoll.drive(&mut self.physics.bodies, elapsed, dt);
            }
            self.physics.step();
            self.accumulator -= FIXED_DT;
        }
        if steps == MAX_STEPS_PER_ADVANCE {
            self.accumulator = self.accumulator.min(FIXED_DT);
        }
        self.accumulator = self.accumulator.max(0.0);
        for tracking in self.ragdolls.values_mut().filter_map(|r| r.tracking.as_mut()) {
            tracking.start.clone_from(&tracking.end);
        }
        steps
    }

    fn ragdoll(&self, id: RagdollId) -> &Ragdoll {
        self.ragdolls.get(&id).expect("unknown RagdollId")
    }
}

impl Ragdoll {
    fn body(&self, bone: usize) -> Option<RigidBodyHandle> {
        let slot = self.bones.iter().position(|b| *b == bone)?;
        Some(self.bodies[slot])
    }

    /// Steers bodies toward the target pose at the end of a fixed step that
    /// ends `elapsed` seconds into a frame of `frame` seconds.
    fn drive(&self, bodies: &mut RigidBodySet, elapsed: Real, frame: Real) {
        let Some(tracking) = &self.tracking else {
            return;
        };
        // A step owed from an earlier hitch can run in a frame of zero time.
        let (alpha, rate) = if frame > 0.0 {
            ((elapsed / frame).clamp(0.0, 1.0), 1.0 / frame)
        } else {
            (1.0, 0.0)
        };
        let strength = tracking.strength;
        for ((handle, start), end) in self.bodies.iter().zip(&tracking.start).zip(&tracking.end) {
            let body = &mut bodies[*handle];
            let target = start.lerp(end, alpha);
            // Rapier's linear velocity is that of the center of mass, which
            // sits mid-capsule rather than at the bone origin.
            let com = body.local_center_of_mass();
            let linvel = servo(
                body.linvel(),
                (*end * com - *start * com) * rate,
                target * com - body.center_of_mass(),
                strength,
                MAX_TRACK_SPEED,
                MAX_TRACK_ACCEL,
            );
            let angvel = servo(
                body.angvel(),
                rotation_vector(end.rotation * start.rotation.inverse()) * rate,
                rotation_vector(target.rotation * body.rotation().inverse()),
                strength,
                MAX_TRACK_ANGULAR_SPEED,
                MAX_TRACK_ANGULAR_ACCEL,
            );
            body.set_linvel(linvel, true);
            body.set_angvel(angvel, true);
        }
    }
}

/// One step of a PD drive in velocity form. `feed` is the target's own
/// velocity, `error` how far the body is from where the target will be at the
/// end of the step. Returns the body's new velocity.
fn servo(velocity: Vector, feed: Vector, error: Vector, strength: Real, max_speed: Real, max_accel: Real) -> Vector {
    let accel = strength * max_accel;
    // The error left after the step if the body moved with the target closes
    // with time constant TRACK_TIME / strength. At full strength the body gets
    // the goal velocity within one step unless the acceleration limit holds it.
    let rest = error - feed * FIXED_DT;
    // A far target is approached no faster than half the acceleration limit
    // can brake within the remaining error. Closing at the linear rate, the
    // body would overshoot by its braking distance, and further when gravity
    // takes part of the limit.
    let closing = (rest * (strength / TRACK_TIME)).clamp_length_max((accel * rest.length()).sqrt());
    let goal = (feed + closing).clamp_length_max(max_speed);
    velocity + ((goal - velocity) * strength).clamp_length_max(accel * FIXED_DT)
}

/// Rotation as axis times angle along the shorter arc: `q` and `-q` are the
/// same rotation, and animation data may switch between them.
fn rotation_vector(rotation: Rotation) -> Vector {
    if rotation.w < 0.0 { -rotation } else { rotation }.to_scaled_axis()
}
