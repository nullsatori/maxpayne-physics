# Max Payne Passthrough

Max Payne 1 renders, Max Payne 3 animates. Both games run at once on the
user's Windows PC: `mp3bridge.asi` drives MP3 puppets (animation, Euphoria)
through Script Hook and streams their bone poses over shared memory to
`mp1phys.asi`, which will retarget them onto MP1 characters and keep them out
of MP1 walls with Rapier. Crate map, install steps, expected log lines and
the probe files: [README.md](README.md) (Russian).

## Environment

- Max Payne 1: disc install, `C:\Program Files (x86)\Max Payne`,
  `MaxPayne.exe` 1.0.4.0, 32-bit, no DRM. Ultimate ASI Loader is installed
  as `d3d8.dll` and loads `scripts\*.asi` (WidescreenFix is already there).
- Max Payne 3: Steam, `...\steamapps\common\Max Payne 3\Max Payne 3\`,
  `MaxPayne3.exe` 1.0.0.276, 32-bit, `.tbm` section encrypted. Script Hook
  1.0.21.0 (`ScriptHook.dll` + ASI-loading `dinput8.dll`) is installed; it
  officially supports Steam 1.0.0.216 and 1.0.0.255 only. Single player
  only; never run mods in multiplayer.
- Game files (binaries, archives, extracted assets, memory dumps) are never
  committed. Code, notes and small text dumps of names/offsets are fine.

## What is known

- MP1 engine DLLs export MSVC C++ symbols: `e2mfc.dll` (P_VirtualObject,
  P_Scene, P_Driver, P_Camera, P_CollisionBuffer, ...), `rlmfc.dll`,
  `e2driver\e2_d3d8_driver_mfc.dll`, `sndmfc.dll`, `grphmfc.dll`.
- `MaxPayne.exe` imports `P_VirtualObject::setBoneMatrices(const
  std::rv<const M_Matrix4x3Template<float>*>&)` from `e2mfc.dll`: skinned
  characters hand their bone matrices to the renderer there, so it is the
  planned place to substitute poses. `mp1phys.asi` hooks it through the IAT
  and logs samples; the layouts of `std::rv` and `M_Matrix4x3Template<float>`
  are still unknown.
- `MaxPayne.exe` carries RTTI (`.?AVMaxPayne_GameModeMessage_EnableBulletTime@@`,
  `.?AVKF_ObjectKeyframeAnimationCallback@KF2@@`, ...); Ghidra's RTTI
  analysis recovers classes and vtables. Useful strings:
  `TwoHandedAimingPivotBone`, `DeathAngleRightFootBoneName`, `OnDeath`,
  `GM_EnableBulletTime`, `GM_ResetShootDodgeBulletTimeCounters`.
- Script Hook API and natives come from the public plugin SDK
  (github.com/alloc8or/mp3-plugin-sdk, no license file: reference hashes and
  tags, do not vendor its files). `crates/scripthook` binds the API at run
  time and verifies each export's MSVC decoration. Natives run only on the
  script fiber registered with `scriptRegister`.
- `GET_PED_BONE_COORDS(ped, bone, offset)` is the pose source on the MP3
  side. Whether the offset is bone-local (then three samples per bone give
  its rotation) is decided at run time by `crates/mp3/src/frames.rs` and
  logged as a verdict. Ped-axis offsets would also read as bone-local; then
  every local rotation in `mp3bridge.player-skeleton.txt` is identity.

## Next work (needs the games)

In order; each step unblocks the next. Record findings with evidence (log
excerpts, function names, how a value was confirmed) in `docs/findings.md`.

1. **First runs.** Follow README "Пробы (этап 2)" and "Max Payne 3". Check
   that Script Hook works on 1.0.0.276, that `scriptRegister` from the plugin
   thread starts the script, the offset verdict, and the skeleton dump.
2. **MP1 bone data layout.** From the `setBoneMatrices` records pick the `rv`
   interpretation and the matrix layout (`pose::matrix::classify`). Settle
   the rotation transpose by experiment: override one bone (e.g. turn the
   head 45° about a known axis) and look at the screen.
3. **Which object is who.** Map `P_VirtualObject` instances to characters
   (player vs enemies) via RTTI classes in `MaxPayne.exe`. Find a per-frame
   point to drive MP1's heartbeat and pose updates (MP3's already beats per
   script frame).
4. **MP1 skeleton.** Dump bone names, parents and rest pose in the
   `pose::Skeleton` text format, at run time or from
   `data\database\skeletons\default_skeleton` in the `.ras` archives. Then
   write the MP3 -> MP1 mapping (`pose::retarget::Mapping`) and the
   coordinate conversion (MP3 is Z-up metres).
5. **Events.** Locate hits (bone, point, direction), deaths and the bullet
   time scale in MP1, and send `bridge::Command`s.
6. **Apply.** Retarget streamed poses, drive `ragdoll::RagdollWorld::track`
   against MP1 level collision, encode with the confirmed matrix layout and
   substitute them in the hook.

## Open issues to settle before MP1 sends commands

- MP3 skips draining commands while there is no main player (loading,
  cutscenes); a full ring then makes MP1's `send` fail and stale commands
  run later.
- A cancelled or timed-out spawn releases a model another puppet slot may
  still be loading.
- `SET_TIME_SCALE` stays in effect if MP1 disconnects during bullet time.
- Undocumented native parameters (CREATE_PED ped type, SET_PED_TO_RAGDOLL
  timings, APPLY_FORCE_TO_PED force type and whether its bone argument is an
  ePedBone tag) are guesses, logged on every call.
- A non-finite tracking target permanently corrupts a ragdoll body: validate
  streamed poses (finite, unit quaternions, counts) before `track`.

## Rules

- One mechanism at a time. When an assumption about game data is confirmed,
  replace the probe or heuristic with the confirmed path and note the
  evidence; do not keep both.
- Code inside a game must never crash it: check memory with
  `winmod::readable` before dereferencing unconfirmed pointers, call the
  original in hooks, keep per-frame work cheap, rate-limit logging, catch
  panics at every FFI entry.
- Keep logic that does not need Windows in host-testable modules with tests
  (`pose`, `ragdoll`, `bridge`, `scripthook` core, pure modules in `mp1` and
  `mp3`).
- Checks before every push (CI runs the latest stable Rust, so new clippy
  lints can appear there first):
  ```
  cargo fmt --check
  cargo clippy --workspace --all-targets -- -D warnings
  cargo clippy --workspace --all-targets --target i686-pc-windows-msvc -- -D warnings
  cargo test --workspace
  ```
  A change to in-game behavior also needs a run in the game, with the log
  excerpt in the PR. Type-checking is not evidence that a hook works.
- Code comments and commit messages in English; README stays in Russian.
