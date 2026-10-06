//! Native calls the way the SDK's `nativeCaller.h` makes them:
//! `nativeInit(hash)`, one `nativePush32` per argument, then `nativeCall()`,
//! whose returned pointer holds the result.

/// SDK `Hash`: unsigned long, 32 bits on Windows.
pub type Hash = u32;
/// SDK `Ped`: int.
pub type Ped = i32;
/// SDK `Player`: unsigned int.
pub type Player = u32;

/// The three Script Hook calls behind every native.
///
/// # Safety
/// The pointer `call` returns must be null or valid for reading the result
/// of the native just called, up to three `u32` (a vector), until the next
/// `init`.
pub unsafe trait Backend {
    fn init(&mut self, hash: Hash);
    fn push(&mut self, value: u32);
    fn call(&mut self) -> *const u32;
}

/// A native argument as the 32-bit word `nativePush` writes for it.
trait Arg {
    fn word(self) -> u32;
}

impl Arg for u32 {
    fn word(self) -> u32 {
        self
    }
}

impl Arg for i32 {
    fn word(self) -> u32 {
        self as u32
    }
}

/// `BOOL`.
impl Arg for bool {
    fn word(self) -> u32 {
        self.into()
    }
}

impl Arg for f32 {
    fn word(self) -> u32 {
        self.to_bits()
    }
}

/// A pointer fits one argument only on a 32-bit target, like the game.
#[cfg(target_pointer_width = "32")]
impl<T> Arg for *mut T {
    fn word(self) -> u32 {
        self as usize as u32
    }
}

/// Most words a result is read as (`Vector3`).
const MAX_RESULT_WORDS: usize = 3;

/// A native result, decoded from the words `nativeCall`'s pointer holds.
trait Ret {
    /// Words read; 0 when the result is not read.
    const WORDS: usize;
    fn decode(words: &[u32]) -> Self;
}

impl Ret for () {
    const WORDS: usize = 0;
    fn decode(_: &[u32]) {}
}

impl Ret for u32 {
    const WORDS: usize = 1;
    fn decode(words: &[u32]) -> Self {
        words[0]
    }
}

impl Ret for i32 {
    const WORDS: usize = 1;
    fn decode(words: &[u32]) -> Self {
        words[0] as i32
    }
}

/// `BOOL`: any nonzero value is true.
impl Ret for bool {
    const WORDS: usize = 1;
    fn decode(words: &[u32]) -> Self {
        words[0] != 0
    }
}

impl Ret for f32 {
    const WORDS: usize = 1;
    fn decode(words: &[u32]) -> Self {
        f32::from_bits(words[0])
    }
}

/// `Vector3`: three consecutive floats.
impl Ret for [f32; 3] {
    const WORDS: usize = 3;
    fn decode(words: &[u32]) -> Self {
        [0, 1, 2].map(|i| f32::from_bits(words[i]))
    }
}

/// The natives the plugin calls, over a [`Backend`], counting calls.
/// Each one's doc names its SDK namespace, hash and declaration.
pub struct Natives<B> {
    backend: B,
    calls: u32,
}

impl<B: Backend> Natives<B> {
    pub fn new(backend: B) -> Self {
        Self { backend, calls: 0 }
    }

    pub fn backend(&self) -> &B {
        &self.backend
    }

    /// Native calls since the previous call of this function.
    pub fn take_calls(&mut self) -> u32 {
        std::mem::take(&mut self.calls)
    }

    fn invoke<R: Ret>(&mut self, hash: Hash, args: &[u32]) -> R {
        const { assert!(R::WORDS <= MAX_RESULT_WORDS) };
        self.calls += 1;
        self.backend.init(hash);
        for &word in args {
            self.backend.push(word);
        }
        let result = self.backend.call();
        let mut words = [0u32; MAX_RESULT_WORDS];
        if R::WORDS > 0 {
            assert!(!result.is_null(), "native {hash:#010x} returned no result");
            for (i, word) in words[..R::WORDS].iter_mut().enumerate() {
                // SAFETY: Backend's contract. The SDK reads the result in
                // place; nothing documents its alignment.
                *word = unsafe { result.add(i).read_unaligned() };
            }
        }
        R::decode(&words[..R::WORDS])
    }

    /// `PLAYER` 0x2528378B: `BOOL DOES_MAIN_PLAYER_EXIST()`.
    pub fn does_main_player_exist(&mut self) -> bool {
        self.invoke(0x2528_378B, &[])
    }

    /// `PLAYER` 0x1CC648EA: `Player GET_PLAYER_ID()`.
    pub fn get_player_id(&mut self) -> Player {
        self.invoke(0x1CC6_48EA, &[])
    }

    /// `PLAYER` 0x6E31E993: `Ped GET_PLAYER_PED(Player player)`.
    pub fn get_player_ped(&mut self, player: Player) -> Ped {
        self.invoke(0x6E31_E993, &[player.word()])
    }

    /// `PED` 0xF7C8D072: `BOOL DOES_PED_EXIST(Ped ped)`.
    pub fn does_ped_exist(&mut self, ped: Ped) -> bool {
        self.invoke(0xF7C8_D072, &[ped.word()])
    }

    /// `PED` 0x4579CAB1: `Vector3 GET_PED_BONE_COORDS(Ped ped, int boneId,
    /// float offsetX, float offsetY, float offsetZ)`. `bone_id` is an
    /// `ePedBone` tag.
    pub fn get_ped_bone_coords(&mut self, ped: Ped, bone_id: i32, offset: [f32; 3]) -> [f32; 3] {
        let [x, y, z] = offset;
        self.invoke(0x4579_CAB1, &[ped.word(), bone_id.word(), x.word(), y.word(), z.word()])
    }

    /// `SYSTEM` 0x50597EE2: `float TIMESTEP()`.
    pub fn timestep(&mut self) -> f32 {
        self.invoke(0x5059_7EE2, &[])
    }

    /// `MISC` 0xA7F84694: `void SET_TIME_SCALE(float scale)`.
    pub fn set_time_scale(&mut self, scale: f32) {
        self.invoke(0xA7F8_4694, &[scale.word()])
    }

    /// `STREAMING` 0xFFF1B500: `Request_s REQUEST_MODEL(Hash modelHash)`.
    /// The returned request is not read.
    pub fn request_model(&mut self, model: Hash) {
        self.invoke(0xFFF1_B500, &[model.word()])
    }

    /// `STREAMING` 0x62BFDB37: `BOOL HAS_MODEL_LOADED(Hash modelHash)`.
    pub fn has_model_loaded(&mut self, model: Hash) -> bool {
        self.invoke(0x62BF_DB37, &[model.word()])
    }

    /// `STREAMING` 0xAE0F069E: `void SET_MODEL_AS_NO_LONGER_NEEDED(Hash modelHash)`.
    pub fn set_model_as_no_longer_needed(&mut self, model: Hash) {
        self.invoke(0xAE0F_069E, &[model.word()])
    }

    /// `PED` 0x0389EF71: `Ped CREATE_PED(int pedType, Hash modelHash, float x,
    /// float y, float z, float heading, BOOL isNetwork, BOOL dontOwn)`.
    pub fn create_ped(
        &mut self,
        ped_type: i32,
        model: Hash,
        position: [f32; 3],
        heading: f32,
        is_network: bool,
        dont_own: bool,
    ) -> Ped {
        let [x, y, z] = position;
        let args = [
            ped_type.word(),
            model.word(),
            x.word(),
            y.word(),
            z.word(),
            heading.word(),
            is_network.word(),
            dont_own.word(),
        ];
        self.invoke(0x0389_EF71, &args)
    }

    /// `PED` 0x13EFB9A0: `void DELETE_PED(Ped* ped)`.
    #[cfg(target_pointer_width = "32")]
    pub fn delete_ped(&mut self, ped: &mut Ped) {
        self.invoke(0x13EF_B9A0, &[std::ptr::from_mut(ped).word()])
    }

    /// `PED` 0x1FC771E2: `void SET_PED_INVINCIBLE(Ped ped, BOOL toggle)`.
    pub fn set_ped_invincible(&mut self, ped: Ped, toggle: bool) {
        self.invoke(0x1FC7_71E2, &[ped.word(), toggle.word()])
    }

    /// `PED` 0x343ABB0E: `void APPLY_FORCE_TO_PED(Ped ped, int forceType,
    /// float forceX, float forceY, float forceZ, float offX, float offY,
    /// float offZ, int bone, BOOL isDirectionRel, BOOL ignoreUpVec,
    /// BOOL isForceRel)`.
    #[allow(clippy::too_many_arguments)] // The native's own parameters.
    pub fn apply_force_to_ped(
        &mut self,
        ped: Ped,
        force_type: i32,
        force: [f32; 3],
        offset: [f32; 3],
        bone: i32,
        is_direction_rel: bool,
        ignore_up_vec: bool,
        is_force_rel: bool,
    ) {
        let ([fx, fy, fz], [ox, oy, oz]) = (force, offset);
        let args = [
            ped.word(),
            force_type.word(),
            fx.word(),
            fy.word(),
            fz.word(),
            ox.word(),
            oy.word(),
            oz.word(),
            bone.word(),
            is_direction_rel.word(),
            ignore_up_vec.word(),
            is_force_rel.word(),
        ];
        self.invoke(0x343A_BB0E, &args)
    }

    /// `PED` 0x83CB5052: `BOOL SET_PED_TO_RAGDOLL(Ped ped, int time1, int time2,
    /// int p3, BOOL p4, BOOL p5, BOOL p6, float p7)`.
    #[allow(clippy::too_many_arguments)] // The native's own parameters.
    pub fn set_ped_to_ragdoll(
        &mut self,
        ped: Ped,
        time1: i32,
        time2: i32,
        p3: i32,
        p4: bool,
        p5: bool,
        p6: bool,
        p7: f32,
    ) -> bool {
        let args = [
            ped.word(),
            time1.word(),
            time2.word(),
            p3.word(),
            p4.word(),
            p5.word(),
            p6.word(),
            p7.word(),
        ];
        self.invoke(0x83CB_5052, &args)
    }
}

type Answer = Box<dyn FnMut(Hash, &[u32]) -> [u32; MAX_RESULT_WORDS]>;

/// A [`Backend`] for tests: records the hash and arguments of every native
/// and answers each from a closure given them.
pub struct Recorder {
    pub calls: Vec<(Hash, Vec<u32>)>,
    answer: Answer,
    result: [u32; MAX_RESULT_WORDS],
}

impl Recorder {
    pub fn new(answer: impl FnMut(Hash, &[u32]) -> [u32; MAX_RESULT_WORDS] + 'static) -> Self {
        Self {
            calls: Vec::new(),
            answer: Box::new(answer),
            result: [0; MAX_RESULT_WORDS],
        }
    }
}

// SAFETY: the result lives in the Recorder until the next call.
unsafe impl Backend for Recorder {
    fn init(&mut self, hash: Hash) {
        self.calls.push((hash, Vec::new()));
    }

    fn push(&mut self, value: u32) {
        self.calls
            .last_mut()
            .expect("nativePush32 before nativeInit")
            .1
            .push(value);
    }

    fn call(&mut self) -> *const u32 {
        let (hash, args) = self.calls.last().expect("nativeCall before nativeInit");
        self.result = (self.answer)(*hash, args);
        self.result.as_ptr()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bits(values: [f32; 3]) -> [u32; 3] {
        values.map(f32::to_bits)
    }

    #[test]
    fn bone_coords_push_in_declaration_order_and_decode_a_vector() {
        let mut natives = Natives::new(Recorder::new(|_, _| bits([1.5, -2.25, 1000.125])));
        let coords = natives.get_ped_bone_coords(0x0102, 0xC018, [0.1, 0.0, -0.5]);
        assert_eq!(coords, [1.5, -2.25, 1000.125]);
        assert_eq!(
            natives.backend().calls,
            [(
                0x4579_CAB1,
                vec![0x0102, 0xC018, 0.1f32.to_bits(), 0, (-0.5f32).to_bits()]
            )]
        );
    }

    #[test]
    fn scalar_results_decode() {
        let mut answers = [
            [0x3D08_8889, 0, 0],
            [2, 0, 0],
            [0, 0, 0],
            [0xFFFF_FFFE, 0, 0],
            [7, 0, 0],
        ]
        .into_iter();
        let mut natives = Natives::new(Recorder::new(move |_, _| answers.next().unwrap()));
        assert_eq!(natives.timestep(), f32::from_bits(0x3D08_8889));
        assert!(natives.does_main_player_exist(), "any nonzero BOOL is true");
        assert!(!natives.has_model_loaded(0x647F_FDD9));
        assert_eq!(natives.get_player_ped(3), -2);
        assert_eq!(natives.get_player_id(), 7);
        let hashes: Vec<Hash> = natives.backend().calls.iter().map(|call| call.0).collect();
        assert_eq!(
            hashes,
            [0x5059_7EE2, 0x2528_378B, 0x62BF_DB37, 0x6E31_E993, 0x1CC6_48EA]
        );
        assert_eq!(natives.backend().calls[3].1, [3]);
        assert_eq!(natives.take_calls(), 5);
        assert_eq!(natives.take_calls(), 0);
    }

    #[test]
    fn bools_and_floats_encode() {
        let mut natives = Natives::new(Recorder::new(|_, _| [1, 0, 0]));
        let ped = natives.create_ped(4, 0x647F_FDD9, [1.0, 2.0, -3.0], 90.0, false, true);
        assert_eq!(ped, 1);
        natives.set_ped_invincible(ped, true);
        natives.apply_force_to_ped(ped, 1, [0.0, 50.0, -1.0], [0.5, 0.0, 0.0], 0xA555, false, true, false);
        assert!(natives.set_ped_to_ragdoll(ped, 1000, 3000, 0, true, false, false, 0.25));
        let f = |v: f32| v.to_bits();
        assert_eq!(
            natives.backend().calls,
            [
                (
                    0x0389_EF71,
                    vec![4, 0x647F_FDD9, f(1.0), f(2.0), f(-3.0), f(90.0), 0, 1]
                ),
                (0x1FC7_71E2, vec![1, 1]),
                (
                    0x343A_BB0E,
                    vec![1, 1, 0, f(50.0), f(-1.0), f(0.5), 0, 0, 0xA555, 0, 1, 0]
                ),
                (0x83CB_5052, vec![1, 1000, 3000, 0, 1, 0, 0, f(0.25)]),
            ]
        );
    }

    #[cfg(target_pointer_width = "32")]
    #[test]
    fn pointer_arguments_pass_the_address() {
        let mut natives = Natives::new(Recorder::new(|_, _| [0; 3]));
        let mut ped: Ped = 5;
        let address = &raw mut ped as usize as u32;
        natives.delete_ped(&mut ped);
        assert_eq!(natives.backend().calls, [(0x13EF_B9A0, vec![address])]);
    }

    /// A Script Hook that returns no result pointer.
    struct NoResult;

    // SAFETY: returns null, which Backend allows.
    unsafe impl Backend for NoResult {
        fn init(&mut self, _: Hash) {}
        fn push(&mut self, _: u32) {}
        fn call(&mut self) -> *const u32 {
            std::ptr::null()
        }
    }

    #[test]
    fn natives_without_a_result_ignore_the_pointer() {
        let mut natives = Natives::new(NoResult);
        natives.set_time_scale(0.2);
        natives.request_model(0x647F_FDD9);
        natives.set_model_as_no_longer_needed(0x647F_FDD9);
    }

    #[test]
    #[should_panic(expected = "native 0x4579cab1 returned no result")]
    fn a_missing_result_is_not_read() {
        Natives::new(NoResult).get_ped_bone_coords(1, 0, [0.0; 3]);
    }
}
