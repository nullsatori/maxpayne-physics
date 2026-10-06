//! Game-independent ragdoll simulation.
//!
//! Units are meters and seconds with +Y up; the game adapter converts game
//! units and axes before calling in. Each ragdoll body is attached to one
//! skeleton bone: the rigid body pose *is* the bone's world pose, so reading
//! a pose back needs no retargeting offsets.

use rapier3d::prelude::*;

/// Fixed simulation step. Game frames are split into steps of this size so
/// the result does not depend on frame rate or bullet time scale.
pub const FIXED_DT: Real = 1.0 / 120.0;

/// Upper bound on fixed steps per `advance` call, so a long hitch does not
/// stall the game; the remaining time is dropped.
pub const MAX_STEPS_PER_ADVANCE: u32 = 16;

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
    /// Bone index into the pose arrays passed to [`Ragdoll::spawn`].
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

pub struct RagdollWorld {
    pub physics: PhysicsWorld,
    accumulator: Real,
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

    /// Advances by `dt` seconds of game time. Bullet time is expressed by the
    /// game passing scaled time, which keeps every step at `FIXED_DT`.
    /// Returns the number of fixed steps taken.
    pub fn advance(&mut self, dt: Real) -> u32 {
        // Tolerance absorbs float error when frames sum to whole steps.
        const EPS: Real = 1.0e-5;
        self.accumulator += dt.max(0.0);
        let mut steps = 0;
        while self.accumulator + EPS >= FIXED_DT && steps < MAX_STEPS_PER_ADVANCE {
            self.physics.step();
            self.accumulator -= FIXED_DT;
            steps += 1;
        }
        if steps == MAX_STEPS_PER_ADVANCE {
            self.accumulator = self.accumulator.min(FIXED_DT);
        }
        self.accumulator = self.accumulator.max(0.0);
        steps
    }
}

pub struct Ragdoll {
    bones: Vec<usize>,
    bodies: Vec<RigidBodyHandle>,
}

impl Ragdoll {
    /// Creates bodies matching the current animated pose.
    ///
    /// `bind` holds bind-pose bone transforms in model space and defines the
    /// neutral joint orientation that limits are measured from. `pose` holds
    /// current world transforms. `previous`, when given, is the world pose
    /// `dt` seconds earlier and lets the ragdoll inherit animation momentum.
    pub fn spawn(
        world: &mut RagdollWorld,
        defs: &[BodyDef],
        bind: &[Pose],
        pose: &[Pose],
        previous: Option<(&[Pose], Real)>,
    ) -> Self {
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
                let delta = current.rotation * before.rotation.inverse();
                builder = builder
                    .linvel((current.translation - before.translation) / dt)
                    .angvel(delta.to_scaled_axis() / dt);
            }
            let handle = world.physics.insert_body(builder);
            let collider = ColliderBuilder::capsule_from_endpoints(Vector::ZERO, def.end, def.radius)
                .density(def.density)
                .friction(0.7);
            world.physics.insert_collider(collider, Some(handle));

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
                world.physics.insert_impulse_joint(bodies[parent], handle, joint);
            }
            bodies.push(handle);
        }
        Self {
            bones: defs.iter().map(|d| d.bone).collect(),
            bodies,
        }
    }

    /// Current world pose of every simulated bone, as `(bone index, pose)`.
    pub fn bone_poses(&self, world: &RagdollWorld) -> Vec<(usize, Pose)> {
        self.bones
            .iter()
            .zip(&self.bodies)
            .map(|(bone, handle)| (*bone, *world.physics.bodies[*handle].position()))
            .collect()
    }

    /// Applies a bullet hit at a world point to the body of `bone`.
    /// Returns false when the bone is not part of this ragdoll.
    pub fn apply_impulse(&self, world: &mut RagdollWorld, bone: usize, point: Vector, impulse: Vector) -> bool {
        let Some(slot) = self.bones.iter().position(|b| *b == bone) else {
            return false;
        };
        world.physics.bodies[self.bodies[slot]].apply_impulse_at_point(impulse, point, true);
        true
    }

    pub fn linear_velocity(&self, world: &RagdollWorld, bone: usize) -> Option<Vector> {
        let slot = self.bones.iter().position(|b| *b == bone)?;
        Some(world.physics.bodies[self.bodies[slot]].linvel())
    }

    pub fn despawn(self, world: &mut RagdollWorld) {
        for handle in self.bodies {
            world.physics.remove_body(handle);
        }
    }
}
