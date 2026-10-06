//! Shared-memory link between the two game processes.
//!
//! Max Payne 3 runs the puppets (animation and Euphoria) and publishes their
//! bone poses; Max Payne 1 sends commands (spawn, hit, time scale) and reads
//! the poses back. Both sides map one named region whose layout is
//! [`Shared`]:
//!
//! * poses travel through a seqlock: one writer (MP3), readers copy a whole
//!   frame and retry if the writer touched it meanwhile;
//! * commands travel through a single-producer single-consumer ring (MP1 to MP3);
//! * each side bumps a heartbeat counter so the other can tell it is alive.
//!
//! The region starts zeroed, which is a valid empty state for every field.

use std::cell::UnsafeCell;
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering, fence};

#[cfg(windows)]
mod mapping;

pub const MAPPING_NAME: &str = "Local\\MaxPaynePassthrough";
pub const MAGIC: u32 = u32::from_le_bytes(*b"MPPT");
/// Bump on any change to the layout or meaning of shared data.
/// v2: [`ActorPose::flags`].
pub const VERSION: u32 = 2;
pub const MAX_ACTORS: usize = 32;
pub const MAX_BONES: usize = 128;
pub const COMMAND_CAPACITY: u32 = 256;

/// Bone transform in the source game's world space; rotation is `[x, y, z, w]`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BoneTransform {
    pub translation: [f32; 3],
    pub rotation: [f32; 4],
}

/// One actor's bones. Max Payne 3 actors list their bones in the order of
/// the MP3 body bone table (`scripthook::BODY_BONES`).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActorPose {
    pub actor: u32,
    pub bone_count: u32,
    /// [`ActorPose::ROTATIONS_VALID`]; readers ignore bits they do not know.
    pub flags: u32,
    pub bones: [BoneTransform; MAX_BONES],
}

impl ActorPose {
    /// The bone rotations were measured. Without it only the translations
    /// are, and the rotations are identity placeholders.
    pub const ROTATIONS_VALID: u32 = 1 << 0;

    pub fn new(actor: u32, flags: u32, bones: &[BoneTransform]) -> Self {
        assert!(bones.len() <= MAX_BONES, "too many bones: {}", bones.len());
        let mut pose = Self {
            actor,
            bone_count: bones.len() as u32,
            flags,
            bones: [BoneTransform::default(); MAX_BONES],
        };
        pose.bones[..bones.len()].copy_from_slice(bones);
        pose
    }

    pub fn bones(&self) -> &[BoneTransform] {
        &self.bones[..self.bone_count as usize]
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PoseFrame {
    /// Increments with every published frame; 0 means nothing published yet.
    pub frame: u64,
    /// MP3 game time of the frame, seconds.
    pub game_time: f64,
    pub actor_count: u32,
    _pad: u32,
    pub actors: [ActorPose; MAX_ACTORS],
}

impl PoseFrame {
    /// An empty frame on the heap (the struct is too large for game thread stacks).
    pub fn new_boxed() -> Box<Self> {
        let layout = std::alloc::Layout::new::<Self>();
        // SAFETY: all-zero bytes are a valid PoseFrame.
        let ptr = unsafe { std::alloc::alloc_zeroed(layout) } as *mut Self;
        if ptr.is_null() {
            std::alloc::handle_alloc_error(layout);
        }
        // SAFETY: allocated with the global allocator and Self's layout.
        unsafe { Box::from_raw(ptr) }
    }

    pub fn actors(&self) -> &[ActorPose] {
        &self.actors[..self.actor_count as usize]
    }
}

/// Requests from Max Payne 1 to Max Payne 3. Actor ids are chosen by MP1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Command {
    /// Create a puppet for an MP1 character; `kind` selects the MP3 model.
    Spawn {
        actor: u32,
        kind: u32,
    },
    Despawn {
        actor: u32,
    },
    /// A bullet hit, in MP3 world space after MP1-to-MP3 conversion. `bone`
    /// indexes the source (MP3) body bone table, `scripthook::BODY_BONES`.
    Hit {
        actor: u32,
        bone: u32,
        point: [f32; 3],
        impulse: [f32; 3],
    },
    /// Game speed, 1.0 normal; follows MP1 bullet time.
    TimeScale {
        scale: f32,
    },
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct RawCommand {
    kind: u32,
    actor: u32,
    arg: u32,
    _pad: u32,
    values: [f32; 6],
}

const CMD_SPAWN: u32 = 1;
const CMD_DESPAWN: u32 = 2;
const CMD_HIT: u32 = 3;
const CMD_TIME_SCALE: u32 = 4;

impl Command {
    fn encode(self) -> RawCommand {
        let mut raw = RawCommand::default();
        match self {
            Command::Spawn { actor, kind } => (raw.kind, raw.actor, raw.arg) = (CMD_SPAWN, actor, kind),
            Command::Despawn { actor } => (raw.kind, raw.actor) = (CMD_DESPAWN, actor),
            Command::Hit {
                actor,
                bone,
                point,
                impulse,
            } => {
                (raw.kind, raw.actor, raw.arg) = (CMD_HIT, actor, bone);
                raw.values[..3].copy_from_slice(&point);
                raw.values[3..].copy_from_slice(&impulse);
            }
            Command::TimeScale { scale } => {
                raw.kind = CMD_TIME_SCALE;
                raw.values[0] = scale;
            }
        }
        raw
    }

    fn decode(raw: RawCommand) -> Option<Self> {
        let v = raw.values;
        Some(match raw.kind {
            CMD_SPAWN => Command::Spawn {
                actor: raw.actor,
                kind: raw.arg,
            },
            CMD_DESPAWN => Command::Despawn { actor: raw.actor },
            CMD_HIT => Command::Hit {
                actor: raw.actor,
                bone: raw.arg,
                point: [v[0], v[1], v[2]],
                impulse: [v[3], v[4], v[5]],
            },
            CMD_TIME_SCALE => Command::TimeScale { scale: v[0] },
            _ => return None,
        })
    }
}

#[repr(C)]
pub struct Shared {
    magic: AtomicU32,
    version: AtomicU32,
    mp1_pid: AtomicU32,
    mp3_pid: AtomicU32,
    mp1_heartbeat: AtomicU64,
    mp3_heartbeat: AtomicU64,
    pose_seq: AtomicU32,
    _pad0: u32,
    pose: UnsafeCell<PoseFrame>,
    cmd_head: AtomicU32,
    cmd_tail: AtomicU32,
    commands: UnsafeCell<[RawCommand; COMMAND_CAPACITY as usize]>,
}

// Both games map the same bytes, possibly from 32- and 64-bit builds; the
// layout must not depend on the target. Update these when VERSION changes.
const _: () = assert!(size_of::<ActorPose>() == 3_596);
const _: () = assert!(std::mem::offset_of!(ActorPose, bones) == 12);
const _: () = assert!(size_of::<PoseFrame>() == 115_096);
const _: () = assert!(size_of::<Shared>() == 125_384);
const _: () = assert!(std::mem::offset_of!(Shared, pose) == 40);
const _: () = assert!(std::mem::offset_of!(Shared, cmd_head) == 40 + size_of::<PoseFrame>());

#[derive(Debug, PartialEq)]
pub enum OpenError {
    /// The other side runs a build with a different protocol version.
    VersionMismatch { found: u32 },
    #[cfg(windows)]
    Os { call: &'static str, code: u32 },
}

impl std::fmt::Display for OpenError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            OpenError::VersionMismatch { found } => {
                write!(
                    f,
                    "bridge protocol v{found} found, this build speaks v{VERSION}; update both mods"
                )
            }
            #[cfg(windows)]
            OpenError::Os { call, code } => write!(f, "{call} failed with error {code}"),
        }
    }
}

/// Owns the memory behind a Link; only held, never read.
#[allow(dead_code)]
enum Backing {
    #[cfg(windows)]
    Mapping(mapping::Mapping),
    Heap(std::sync::Arc<HeapRegion>),
}

struct HeapRegion(NonNull<Shared>);

// SAFETY: Shared is only accessed through atomics and the seqlock/ring protocols.
unsafe impl Send for HeapRegion {}
unsafe impl Sync for HeapRegion {}

impl HeapRegion {
    fn new() -> Self {
        let layout = std::alloc::Layout::new::<Shared>();
        // SAFETY: zeroed memory is a valid Shared (atomics, plain data).
        let ptr = unsafe { std::alloc::alloc_zeroed(layout) } as *mut Shared;
        Self(NonNull::new(ptr).unwrap_or_else(|| std::alloc::handle_alloc_error(layout)))
    }
}

impl Drop for HeapRegion {
    fn drop(&mut self) {
        // SAFETY: allocated in `new` with the same layout.
        unsafe { std::alloc::dealloc(self.0.as_ptr() as *mut u8, std::alloc::Layout::new::<Shared>()) }
    }
}

/// An in-process region for tests: returns both ends of one link.
pub fn in_memory_link() -> (Mp1Side, Mp3Side) {
    let region = std::sync::Arc::new(HeapRegion::new());
    let shared = region.0;
    let mp1 = Link::attach(shared, Backing::Heap(region.clone())).expect("fresh region");
    let mp3 = Link::attach(shared, Backing::Heap(region)).expect("fresh region");
    (Mp1Side(mp1), Mp3Side(mp3))
}

struct Link {
    shared: NonNull<Shared>,
    _backing: Backing,
}

// SAFETY: see HeapRegion; a Mapping view is process-wide memory.
unsafe impl Send for Link {}

impl Link {
    fn attach(shared: NonNull<Shared>, backing: Backing) -> Result<Self, OpenError> {
        let link = Self {
            shared,
            _backing: backing,
        };
        let s = link.shared();
        if s.magic.load(Ordering::Acquire) != MAGIC {
            // Both sides may get here at once; they store identical values.
            s.version.store(VERSION, Ordering::Relaxed);
            s.magic.store(MAGIC, Ordering::Release);
        }
        let found = s.version.load(Ordering::Relaxed);
        if found != VERSION {
            return Err(OpenError::VersionMismatch { found });
        }
        Ok(link)
    }

    #[cfg(windows)]
    fn open_named() -> Result<Self, OpenError> {
        let mapping = mapping::Mapping::open(MAPPING_NAME, size_of::<Shared>())?;
        Self::attach(mapping.view(), Backing::Mapping(mapping))
    }

    fn shared(&self) -> &Shared {
        // SAFETY: the backing keeps the region alive for the Link's lifetime.
        unsafe { self.shared.as_ref() }
    }
}

/// Max Payne 1 end: sends commands, reads poses.
pub struct Mp1Side(Link);

/// Max Payne 3 end: receives commands, publishes poses.
pub struct Mp3Side(Link);

impl Mp1Side {
    #[cfg(windows)]
    pub fn open(pid: u32) -> Result<Self, OpenError> {
        let link = Link::open_named()?;
        link.shared().mp1_pid.store(pid, Ordering::Relaxed);
        Ok(Self(link))
    }

    pub fn beat(&self) {
        self.0.shared().mp1_heartbeat.fetch_add(1, Ordering::Release);
    }

    pub fn peer_heartbeat(&self) -> u64 {
        self.0.shared().mp3_heartbeat.load(Ordering::Acquire)
    }

    pub fn peer_pid(&self) -> u32 {
        self.0.shared().mp3_pid.load(Ordering::Relaxed)
    }

    /// Queues a command; returns it back when the ring is full.
    pub fn send(&self, command: Command) -> Result<(), Command> {
        let s = self.0.shared();
        let head = s.cmd_head.load(Ordering::Relaxed);
        let tail = s.cmd_tail.load(Ordering::Acquire);
        if head.wrapping_sub(tail) >= COMMAND_CAPACITY {
            return Err(command);
        }
        // SAFETY: the slot at `head` is not visible to the consumer until
        // cmd_head is published below; Mp1Side is the only producer.
        unsafe {
            let slot = (s.commands.get() as *mut RawCommand).add((head % COMMAND_CAPACITY) as usize);
            slot.write_volatile(command.encode());
        }
        s.cmd_head.store(head.wrapping_add(1), Ordering::Release);
        Ok(())
    }

    /// Copies the latest complete pose frame into `out`. Returns false if
    /// the writer kept the frame busy; `out` then holds garbage.
    pub fn read_poses(&self, out: &mut PoseFrame) -> bool {
        let s = self.0.shared();
        for _ in 0..64 {
            let before = s.pose_seq.load(Ordering::Acquire);
            if before % 2 == 1 {
                std::hint::spin_loop();
                continue;
            }
            // SAFETY: a torn copy is detected by the sequence check below and
            // discarded; PoseFrame is plain data, so any bit pattern is valid.
            unsafe { volatile_copy(s.pose.get(), out) };
            fence(Ordering::Acquire);
            if s.pose_seq.load(Ordering::Relaxed) == before {
                return true;
            }
        }
        false
    }
}

/// Word-wise volatile copy: the source may be written by another process.
unsafe fn volatile_copy<T: Copy>(src: *const T, dst: *mut T) {
    const { assert!(size_of::<T>().is_multiple_of(4) && align_of::<T>() >= 4) };
    let words = size_of::<T>() / 4;
    let (src, dst) = (src as *const u32, dst as *mut u32);
    for i in 0..words {
        // SAFETY: both pointers cover `words` aligned u32 values.
        unsafe { dst.add(i).write(src.add(i).read_volatile()) };
    }
}

impl Mp3Side {
    #[cfg(windows)]
    pub fn open(pid: u32) -> Result<Self, OpenError> {
        let link = Link::open_named()?;
        link.shared().mp3_pid.store(pid, Ordering::Relaxed);
        Ok(Self(link))
    }

    pub fn beat(&self) {
        self.0.shared().mp3_heartbeat.fetch_add(1, Ordering::Release);
    }

    pub fn peer_heartbeat(&self) -> u64 {
        self.0.shared().mp1_heartbeat.load(Ordering::Acquire)
    }

    pub fn peer_pid(&self) -> u32 {
        self.0.shared().mp1_pid.load(Ordering::Relaxed)
    }

    /// Next queued command. Unknown kinds (newer sender) are skipped.
    pub fn receive(&self) -> Option<Command> {
        let s = self.0.shared();
        loop {
            let tail = s.cmd_tail.load(Ordering::Relaxed);
            let head = s.cmd_head.load(Ordering::Acquire);
            if tail == head {
                return None;
            }
            // SAFETY: slots between tail and head are fully written (Acquire
            // on cmd_head); Mp3Side is the only consumer.
            let raw = unsafe {
                (s.commands.get() as *const RawCommand)
                    .add((tail % COMMAND_CAPACITY) as usize)
                    .read_volatile()
            };
            s.cmd_tail.store(tail.wrapping_add(1), Ordering::Release);
            if let Some(command) = Command::decode(raw) {
                return Some(command);
            }
        }
    }

    pub fn publish_poses(&self, game_time: f64, actors: &[ActorPose]) {
        assert!(actors.len() <= MAX_ACTORS, "too many actors: {}", actors.len());
        let s = self.0.shared();
        let seq = s.pose_seq.load(Ordering::Relaxed);
        s.pose_seq.store(seq.wrapping_add(1), Ordering::Relaxed);
        fence(Ordering::Release);
        // SAFETY: Mp3Side is the only writer; readers detect the odd sequence.
        unsafe {
            let frame = s.pose.get();
            let number = (&raw const (*frame).frame).read_volatile();
            (&raw mut (*frame).frame).write_volatile(number + 1);
            (&raw mut (*frame).game_time).write_volatile(game_time);
            (&raw mut (*frame).actor_count).write_volatile(actors.len() as u32);
            let slots = &raw mut (*frame).actors as *mut ActorPose;
            for (i, actor) in actors.iter().enumerate() {
                slots.add(i).write_volatile(*actor);
            }
        }
        s.pose_seq.store(seq.wrapping_add(2), Ordering::Release);
    }
}

/// Tracks whether a peer heartbeat keeps changing.
pub struct Liveness {
    last_value: u64,
    unchanged_ticks: u32,
    timeout_ticks: u32,
}

impl Liveness {
    /// `timeout_ticks`: polls without a new heartbeat before the peer counts as gone.
    pub fn new(timeout_ticks: u32) -> Self {
        Self {
            last_value: 0,
            unchanged_ticks: timeout_ticks,
            timeout_ticks,
        }
    }

    /// Feeds the current peer heartbeat; returns whether the peer is alive.
    pub fn update(&mut self, heartbeat: u64) -> bool {
        if heartbeat != self.last_value {
            self.last_value = heartbeat;
            self.unchanged_ticks = 0;
        } else {
            self.unchanged_ticks = self.unchanged_ticks.saturating_add(1);
        }
        self.unchanged_ticks < self.timeout_ticks
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bone(v: f32) -> BoneTransform {
        BoneTransform {
            translation: [v, v + 1.0, v + 2.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }

    #[test]
    fn commands_round_trip_in_order() {
        let (mp1, mp3) = in_memory_link();
        let sent = [
            Command::Spawn { actor: 7, kind: 2 },
            Command::Hit {
                actor: 7,
                bone: 12,
                point: [1.0, 2.0, 3.0],
                impulse: [-4.0, 5.0, -6.0],
            },
            Command::TimeScale { scale: 0.2 },
            Command::Despawn { actor: 7 },
        ];
        for c in sent {
            mp1.send(c).unwrap();
        }
        let received: Vec<_> = std::iter::from_fn(|| mp3.receive()).collect();
        assert_eq!(received, sent);
    }

    #[test]
    fn full_ring_rejects_and_recovers_across_wraparound() {
        let (mp1, mp3) = in_memory_link();
        for round in 0..3u32 {
            for i in 0..COMMAND_CAPACITY {
                mp1.send(Command::Despawn {
                    actor: round * 1000 + i,
                })
                .unwrap();
            }
            assert_eq!(
                mp1.send(Command::Despawn { actor: 0 }),
                Err(Command::Despawn { actor: 0 })
            );
            for i in 0..COMMAND_CAPACITY {
                assert_eq!(
                    mp3.receive(),
                    Some(Command::Despawn {
                        actor: round * 1000 + i
                    })
                );
            }
            assert_eq!(mp3.receive(), None);
        }
    }

    #[test]
    fn poses_publish_and_read() {
        let (mp1, mp3) = in_memory_link();
        let mut frame = PoseFrame::new_boxed();
        assert!(mp1.read_poses(&mut frame));
        assert_eq!(frame.frame, 0);
        let actors = [
            ActorPose::new(3, ActorPose::ROTATIONS_VALID, &[bone(1.0), bone(2.0)]),
            ActorPose::new(9, 0, &[bone(5.0)]),
        ];
        mp3.publish_poses(12.5, &actors);
        assert!(mp1.read_poses(&mut frame));
        assert_eq!(frame.frame, 1);
        assert_eq!(frame.game_time, 12.5);
        assert_eq!(frame.actors(), &actors);
        assert_eq!(frame.actors()[0].bones(), &[bone(1.0), bone(2.0)]);
        assert_eq!(frame.actors()[0].flags, ActorPose::ROTATIONS_VALID);
        assert_eq!(frame.actors()[1].flags, 0);
    }

    #[test]
    fn concurrent_reader_never_sees_torn_frame() {
        let (mp1, mp3) = in_memory_link();
        let writer = std::thread::spawn(move || {
            for i in 1..=2000u32 {
                let v = i as f32;
                let bones = vec![bone(v); MAX_BONES];
                mp3.publish_poses(v as f64, &[ActorPose::new(i, i, &bones), ActorPose::new(i, i, &bones)]);
            }
        });
        let mut frame = PoseFrame::new_boxed();
        let mut seen = 0;
        while !writer.is_finished() || seen == 0 {
            if !mp1.read_poses(&mut frame) || frame.frame == 0 {
                continue;
            }
            let v = frame.game_time as f32;
            for actor in frame.actors() {
                assert_eq!((actor.actor, actor.flags), (v as u32, v as u32));
                assert!(actor.bones().iter().all(|b| *b == bone(v)), "torn frame at {v}");
            }
            seen += 1;
        }
        writer.join().unwrap();
        assert!(seen > 0);
    }

    #[test]
    fn heartbeats_and_liveness() {
        let (mp1, mp3) = in_memory_link();
        let mut alive = Liveness::new(3);
        assert!(!alive.update(mp1.peer_heartbeat()), "no beats yet");
        mp3.beat();
        assert!(alive.update(mp1.peer_heartbeat()));
        assert!(alive.update(mp1.peer_heartbeat()));
        assert!(alive.update(mp1.peer_heartbeat()));
        assert!(!alive.update(mp1.peer_heartbeat()), "stalled for 3 polls");
        mp1.beat();
        assert_eq!(mp3.peer_heartbeat(), 1);
    }

    #[test]
    fn version_mismatch_is_reported() {
        let region = std::sync::Arc::new(HeapRegion::new());
        // SAFETY: test owns the region.
        let s = unsafe { region.0.as_ref() };
        s.version.store(VERSION + 1, Ordering::Relaxed);
        s.magic.store(MAGIC, Ordering::Release);
        let result = Link::attach(region.0, Backing::Heap(region.clone()));
        assert_eq!(result.err(), Some(OpenError::VersionMismatch { found: VERSION + 1 }));
    }
}
