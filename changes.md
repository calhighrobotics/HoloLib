# Changes

A running log of changes made to this repo through Claude Code sessions. Newest
session at the top. Each entry says what changed, why, and whether it has been
verified on the robot or only compiled.

> **Convention:** this file is updated at the end of every session that touches
> code. Build-only verification is the default, so anything confirmed on the
> real robot is called out explicitly.

---

## 2026-09-05

### 1. Removed `clawGripperPID`/`clawGripperLift` — wrong control model for the mechanism

User corrected a wrong assumption from the 2026-08-29 session: `clawGripper`
(port 10) isn't a gripper jaw that opens/closes to a position — it's a
flex-wheel roller pair that just spins in one direction or the other to
pull/push game pieces. There is no target angle to hold, so a position PID
(`liftlib::Subsystem`) doesn't apply to it at all. Removed the now-provably-wrong
`clawGripperPID`/`clawGripperLift` objects entirely, along with their
`initialize()` call and LCD readout (line 7). The raw `move_voltage` B/Y
hold-buttons already driving this motor were correct all along and are
untouched — nothing to fix there, only something to stop wrapping in the wrong
abstraction.

### 2. Found: the main lift has no closed-loop control active at all

User asked where the lift's PID is. Traced it in
`src/motions/controllers/modular_lift.cpp`: `ModularLift` doesn't use a `PID`
object — it's LQR/state-feedback (`u = -config.K * (x - x_ref)`, `LiftConfig::K`
a 1x2 gain matrix). Not a bug, just a different control law than the claw's.

More importantly: **`my_lift` (the `ModularLift` instance) is never actually
commanded to move.** A repo-wide check of `my_lift.` in `main.cpp` found only
`my_lift.cancel()` in `disabled()`. The lift is actually driven entirely by raw
`move_voltage` on R1/R2 through a separate `pros::MotorGroup liftMotors` on
ports 6/-7. `ModularLift`'s constructor builds its own `pros::Motor` objects on
those same two ports (`modular_lift.cpp:8`) — the same dual-motor-object
pattern flagged and fixed for the claw's pivot back on 2026-08-29, except here
it's currently harmless only because `my_lift` is never told to go anywhere.
### 3. Built the elevator lift's `liftlib::Subsystem`/PID

User pushed back on point 2 — believed a lift PID already existed somewhere in
`liftlib` and asked whether it had been deleted. Verified via a full git-history
grep across every version of `main.cpp` on this branch: no `liftlib::Subsystem`/
`liftlib::PID` for the elevator (ports 6/-7) has ever existed here. `liftlib` is
a generic toolkit, not a pre-populated one — nothing becomes "the elevator's
controller" until something in `main.cpp` constructs a `Subsystem` wired to
those motors, same as `clawRotationLift` had to be built from scratch for the
claw. Built now, following the same skeleton-first pattern: PID + `Subsystem`
declared, `initialize()` (tare) and `setFeedforward()` wired in `initialize()`,
LCD readout added (line 7), **no button wiring yet** — R1/R2 stay on raw
`move_voltage` until this is tuned, same staging as the claw went through.

```cpp
liftlib::PID liftPID(/*kP=*/1.0f, /*kI=*/0.0f, /*kD=*/0.0f, /*threshold=*/2.0f);
liftlib::Subsystem liftLift(
    {{.port = 6,  .gear_ratio = 12.0f/84.0f, ...},
     {.port = -7, .gear_ratio = 12.0f/84.0f, ...}},
    liftPID);
```

Two numbers reused rather than re-measured, since this is the same hardware
`hololib::ModularLift`/`LiftConfig` already describes:
- `gear_ratio = 12.0f/84.0f` is the literal `LiftConfig::gear_ratio` value
  (can't reference that object directly, it's declared later in the file) —
  confirmed by reading `ModularLift::getLiftRadians()` in
  `src/motions/controllers/modular_lift.cpp` that it applies gear_ratio with
  the same output-per-motor-rotation convention `liftlib::Subsystem` uses, so
  it carries over directly.
- `Feedforward::constant(18.5f)`, not `cosine` — a cascade's holding load
  doesn't vary with height the way a pivoting arm's does, unlike the claw.
  `18.5` is a **unit conversion**, not an independent measurement:
  `ModularLift::calculateFeedforward` computes `kG_base * mass * 9.81` in
  millivolts (`1750` for this config), but `liftlib`'s default Voltage
  `OutputMode` wants -127..127, so it's divided by the same `12000/127` scale
  factor used for the joystick elsewhere in this file. Needs verifying the same
  way as the claw's `kG` — watch whether it holds height without sagging or
  climbing.

**Not tuned or tested on hardware yet.** `kP`/`kD` are untuned placeholders
(`1.0f`/`0.0f`) — same oscillation-hunt process as the claw applies here too.
No autonomous test routine was added for it yet (the claw's tuning is still
active in `autonomous()`); ask if one should be added, or wait until the claw
rotation is finished.

### 4. Switched `liftLift` from a single PID to a gain schedule

User confirmed the lift runs on two full motors (ports 6/-7, `liftMotors`,
already correct in point 3) and asked for multiple PIDs across the lift's
travel, the way HoloLib's own `xSched`/`ySched`/`thetaSched` pick gains by
magnitude rather than using one fixed set. `liftlib::Subsystem` has a built-in
equivalent for this — a constructor overload taking
`std::vector<Subsystem::GainPoint>` (`GainPoint = std::pair<PID, float
position>`) instead of a single `PID`. Gains are interpolated linearly between
the two surrounding points and held flat outside the outermost ones; only kP/
kI/kD come from the schedule, everything else (threshold, slew, output limit)
stays a property of the live controller. Replaced the single `liftPID` with:

```cpp
liftlib::Subsystem liftLift(
    {{.port = 6,  .gear_ratio = 12.0f/84.0f, ...},
     {.port = -7, .gear_ratio = 12.0f/84.0f, ...}},
    std::vector<liftlib::Subsystem::GainPoint>{
        {liftlib::PID(1.0f, 0.0f, 0.0f, 2.0f), 0.0f},
        {liftlib::PID(1.0f, 0.0f, 0.0f, 2.0f), 45.0f},
        {liftlib::PID(1.0f, 0.0f, 0.0f, 2.0f), 90.0f},
    });
```

All three points are still the same untuned `1.0f/0.0f/0.0f` placeholder —
this only sets up the *shape* of the schedule, tuning hasn't started. The
0/45/90 positions are placeholders too, and are almost certainly wrong: those
were the claw's pivot range (0-90 degrees), but the lift is a cascade, so its
travel is in output degrees after `gear_ratio` and can span many multiples of
360 depending on `spool_radius` and physical height — not a 90-degree arc.
Before tuning, jog the lift to its real full extension with R1 and read the
LCD's "Lift" line (line 7) to find the actual top-of-travel number, then
replace 45/90 with values that actually span it. Tune bottom-to-top: hold
each point's kP/kD fixed, oscillation-hunt with `moveTo()` near that point's
position, then move up to the next point — expect the top of the range to
want less kP than the bottom, since a fully-extended cascade swings faster
under the same torque. Still no button wiring and no autonomous test routine.

Build verified only (`make quick`, clean compile/link) — not tested on
hardware.

### 5. `liftLift` units: degrees → inches, and a correction to point 3/4's `gear_ratio` reasoning

User asked why the elevator's position was in degrees at all — it's a
cascade/winch (vertical lift for stacking game pieces), so height should read
in inches, not motor-shaft degrees. Correct: degrees only made sense for the
claw's pivot.

While fixing it, found that point 3's claim was wrong: I'd written that
`ModularLift::getLiftRadians()` "confirmed gear_ratio carries over the same
way" — but for `LiftMechanism::CASCADE` specifically, `ModularLift::moveTo()`
targets **raw, unscaled motor degrees** (`controlLoopImpl()` in
`modular_lift.cpp` builds its LQR state `x(pos, vel)` straight from
`motor.get_position()`; `gear_ratio` is never applied to it for a cascade).
`gear_ratio`/`spool_radius` in `LiftConfig` are only read by
`getLiftRadians()`, which only runs for the `FOUR_BAR`/`SIX_BAR`/`VIRTUAL`
feedforward branch — for a cascade they're currently dead fields. So there was
no existing convention to reuse; built the motor-degrees → inches conversion
directly instead:

```cpp
constexpr float LIFT_SPOOL_RADIUS_IN = 1.5f;       // = LiftConfig::spool_radius
constexpr float LIFT_GEAR_RATIO = 12.0f / 84.0f;   // output-rotations-per-motor-rotation
constexpr float LIFT_INCHES_PER_MOTOR_DEGREE =
    LIFT_GEAR_RATIO * (M_PI / 180.0f) * LIFT_SPOOL_RADIUS_IN;
```

used as both motors' `gear_ratio` — arc length = radius × angle-in-radians, so
motor degrees → output degrees (`× 12/84`) → radians (`× π/180`) → inches
(`× spool_radius`). `spool_radius = 1.5f` is reused as a literal from
`LiftConfig` below (same reason `gear_ratio`'s literal was reused: `LiftConfig`
isn't declared yet at this point in the file) — flagged in-code to be measured
directly (half the spool diameter) if 1.5" isn't actually correct for this
robot, since it's a physical constant rather than something to tune.

Also updated the gain schedule's placeholder positions from 0/45/90 (degrees)
to 0/12/24 (inches) and the LCD label to "Lift (in)". Flagged that the
`threshold=2.0f` on all three PID points was picked back when the unit was
degrees — 2 inches of tolerance is probably too loose for a scoring
mechanism and needs revisiting once real gains are being tuned. Unlike the
claw's degree range (which needed jogging-and-reading-the-LCD to find), the
schedule's real positions can be measured directly with a tape measure on the
cascade's physical stroke, retracted to fully extended.

Build verified only (`make quick`, clean compile/link) — not tested on
hardware.

### 6. Replaced borrowed `liftLift` constants with real measurements

User confirmed there's no external gearbox between the lift motors and the
spool (direct drive) and measured the spool radius at 0.4" — both replace the
earlier values that were only ever borrowed literals from `LiftConfig`
(1.5"/12:84), never independently verified for this mechanism:

```cpp
constexpr float LIFT_SPOOL_RADIUS_IN = 0.4f; // measured
constexpr float LIFT_GEAR_RATIO = 1.0f;      // direct drive, no external reduction
```

Also got real travel numbers: claw parallel to the ground reads 6.5" off the
floor at full retraction, 25.5" at full extension — a 19" span. Since
`liftLift.initialize()` tares to 0 wherever the lift sits at boot (assumed to
be fully retracted), the subsystem's own position space runs ~0"-19", not the
real-world 6.5"-25.5" — replaced the gain schedule's placeholder positions
(0"/12"/24") with 0"/9.5"/19" to actually span it, and dropped `threshold`
from the old degrees-era `2.0f` to `1.0f` (still a guess, needs tuning same as
kP/kD once real gains go in).

Build verified only (`make quick`, clean compile/link) — not tested on
hardware.

### 7. Dropped the 0" gain point — the lift never actually targets it

User corrected point 6: the lift is only ever commanded to two heights, 9.5"
and 19" — it's never `moveTo()`'d back to 0" (fully retracted) under PID, so
there's nothing to tune a gain point there for. Removed the 0" `GainPoint`,
leaving just:

```cpp
std::vector<liftlib::Subsystem::GainPoint>{
    {liftlib::PID(1.0f, 0.0f, 0.0f, 1.0f), 9.5f},
    {liftlib::PID(1.0f, 0.0f, 0.0f, 1.0f), 19.0f},
};
```

Two points is still a real schedule — `liftlib` interpolates between whichever
two points surround the live position and holds flat outside them, so this
just means "the same gains apply below 9.5\" as at 9.5\"" rather than needing
a third point to cover territory that's never actually visited under PID
control.

Build verified only (`make quick`, clean compile/link) — not tested on
hardware.

### 8. Added an autonomous test routine for `liftLift`

Added a blocking `liftLift.moveTo(9.5f, ...)` test in `autonomous()`,
directly after the existing claw test — same pattern: `initialize()` right
before the move to re-tare (so each run starts clean, not wherever it drifted
to last), 3s timeout, LCD's "Lift (in)" line shows where it actually stopped.

Direction is flagged as a guess, same caveat as the claw's originally had:
`liftLift`'s ports are `{6, -7}`, the same pair R1/R2 already drive raw in
`opcontrol()` where R1 ("Lift up") sends `+12000` to both — so a positive
`moveTo()` target *should* raise the lift the same way, but this needs
confirming by eye on hardware, not assumed from the LCD number alone. If it's
backwards, the fix is negating both ports' signs in `liftLift`'s
`MotorConfig` (`6 → -6`, `-7 → 7`), not negating the target — that way
`moveTo()` and the R1/R2 raw jog keep pointing the same direction as each
other rather than disagreeing.

Comment also stages the next step once 9.5" is tuned: swap the target to
`19.0f` to tune the top `GainPoint` the same way.

Build verified only (`make quick`, clean compile/link) — **direction not yet
confirmed on hardware.**

### 9. Autonomous now locks the claw before testing the lift

User asked for the claw (motor 9) to be locked in place first, then the lift
tested. Replaced the claw's 90-degree tuning `moveTo()` in `autonomous()`
with `clawRotationLift.initialize(); clawRotationLift.holdActively();` — tares
then actively holds wherever the claw is resting, so it doesn't sag/drift
under gravity while the `liftLift` test below it runs. The original 90-degree
tuning block is preserved directly below, commented out, to switch back to
once the lift test isn't the priority. `autonomous()` now runs: lock claw →
`liftLift.moveTo(9.5f, ...)`.

Build verified only (`make quick`, clean compile/link) — not tested on
hardware.

---

## 2026-08-29

### 1. Repo reorganized to work directly against the org repo

- `origin` repointed from `ashpai2009/HoloLib_Pai` (personal fork) to
  `calhighrobotics/HoloLib` (org repo) directly; `upstream` remote removed since it
  was now redundant.
- Local `main` reset to match `origin/main` exactly.
- New local branch `ashmitBranch`, tracking `origin/ashmitBranch` — this is now
  the only branch worked on and pushed to. `main`/`testing` are off limits.
- **`ashmitBranch` was later reset to `origin/main` at the user's request**,
  discarding this file's previous session (2026-08-21) and the units/heading/
  claw-toggle work from the working tree. That work is not lost — it's still on
  `origin/ashmitBranch` up to commit `ba6ef56` and recoverable from there — but it
  is no longer what's checked out or being built on locally. Anyone picking this
  branch back up should decide explicitly whether to bring it back in, rather
  than assume it's still present.

### 2. `main.cpp` rewritten (by the user, not this session)

Compiles and links. Documented here because `changes.md` tracks repo state, not
just this session's edits — treat this as the new baseline `main.cpp`, replacing
everything described in the 2026-08-21 entry below.

Notable differences from the version this file previously described:
- `clawLift` and `clawRotation` swapped motor ports (9 and 10 reversed).
- All claw controls reverted from single-press toggles back to plain hold
  buttons: B/Y drive `clawLift`, L1/L2 drive `clawRotation`, both at raw
  `move_voltage`. No `tare_position`, no `move_absolute`, no flip state.
- The claw piston (`pros::adi::Pneumatics`) and its B-button binding are gone
  entirely — B now drives `clawLift` instead.
- `ApriltagLocalization`/`tagOdom` removed — no longer constructed or started in
  `initialize()`. **`visionSensor` is no longer defined**, even though
  `config.hpp` still `extern`s it — see the `config.hpp` note added to
  `CLAUDE.md` this session; the build still succeeds only because nothing
  references that symbol.
- `driveControl` call site: `fieldCentric` now `false` (was `true`),
  `headingOffset` now `90` (was `0`), `correctionOn` now `false`. Driver control
  is currently robot-centric, not field-centric.
- Added `pros::misc.h` include and a `pros::lcd::print(6, "test")` debug line at
  the top of `opcontrol`.

### 3. `liftlib` merged into the HoloLib tree

The user had cloned `github.com/Bal207/liftLib` — a separate, independent lift/
subsystem framework (namespace `liftlib`, `Lift`/`Mechanism`/`Subsystem`/`Piston`
classes, PID + feedforward + optional gain-scheduled control) — into a `liftLib/`
subfolder of this repo, as its own nested git clone (own `.git`, own
`project.pros`, and duplicate copies of `include/pros`, `include/liblvgl`,
`firmware/`).

That layout would not have pushed correctly: a nested `.git` is invisible to the
outer repo's normal `git add`, so `liftlib`'s actual source would not have
travelled with `ashmitBranch`. Fixed by merging it in as plain files:

- Copied `liftLib/include/liftlib/*.hpp` → `include/liftlib/`
- Copied `liftLib/src/liftlib/*.cpp` → `src/liftlib/`
- Deleted the `liftLib/` folder entirely, including its nested `.git` and
  duplicated vendor headers/firmware. The original clone was disposable — its
  own remote (`github.com/Bal207/liftLib`) still holds its full history; nothing
  was lost, just de-duplicated.
- Verified `pros build` compiles and links the new `src/liftlib/*.cpp` files
  (the Makefile globs `src/**`, so no build config changes were needed).

**`liftlib::Lift` is not used anywhere in `main.cpp` yet.** It builds as dead
code alongside `hololib::ModularLift`, which is what the robot actually runs.
Documented as a "two unrelated lift frameworks" section in `CLAUDE.md` so a
future session doesn't assume they're connected or interchangeable.

### 4. `CLAUDE.md` corrected and extended

- Added `include/liftlib/**` + `src/liftlib/**` to the Layout section, and a new
  Architecture subsection contrasting `hololib::ModularLift` (LQR/state-feedback,
  what's actually wired into `main.cpp`) against `liftlib::Lift` (PID +
  feedforward, precedence-based stage coordination, not wired in).
- Corrected the `config.hpp` globals claim: omitting a global is only a link
  error when something actually references that symbol, not always. Found via
  the `visionSensor` omission in point 2 above — a global config.hpp `extern`s,
  main.cpp doesn't define, and the build succeeds anyway because nothing calls
  it. The previous wording implied this always fails at link time; it doesn't.
- Note: `CLAUDE.md` had earlier (2026-08-21 session) been corrected to describe
  `main`'s pre-fix behavior after `ashmitBranch` was reset. That correction still
  stands — none of today's `main.cpp` rewrite touched `chassis.cpp` or
  `odometry.cpp`, so the drive-units/`headingOffset`/`correctionOn`/heading-source
  landmines documented there are all still accurate.

### 5. First `liftlib::Subsystem` PID objects for the claw

Compiles and links; **not yet tuned or wired to any button.** Deliberately scoped
this way — the user wants the PIDs working and settled before macros are built
on top of them, and pointed out mid-session that the claw's "flip" mechanism was
replaced with a limited up/down tilt, so its real degrees-of-travel aren't known
yet and no positions were guessed at.

Added near the other claw motor declarations in `src/main.cpp`:

```cpp
liftlib::PID clawRotationPID(0.4f, 0.0f, 0.02f, 2.0f);
liftlib::Subsystem clawRotationLift({{.port = 10, .gear_ratio = 1.0f/12.0f, ...}}, clawRotationPID);

liftlib::PID clawLiftPID(0.4f, 0.0f, 0.02f, 2.0f);
liftlib::Subsystem clawLiftLift({{.port = 9, .gear_ratio = 1.0f/2.7f, ...}}, clawLiftPID);
```

- `LIFTLIB_NO_GLOBAL_NAMES` is defined before including `liftlib/liftlib.hpp`, so
  `liftlib::PID` can't collide with `hololib::PID` if `main.cpp` ever adds a
  `using namespace hololib`.
- Gear ratios: `liftlib::Subsystem` multiplies the raw motor-degree reading by
  `MotorConfig::gear_ratio` to get output degrees, which is the *inverse* of the
  "motor degrees per output degree" convention used in the earlier
  `move_absolute`-based claw code. `clawRotation`'s measured 12:1 became
  `1.0f/12.0f`; `clawLift`'s measured 2.7:1 became `1.0f/2.7f`. Per the user,
  both ratios follow the mechanism/variable name, not the port number, even
  though `clawLift`/`clawRotation`'s port assignments swapped in the main.cpp
  rewrite (point 2 above).
- `clawRotationLift.initialize()` / `clawLiftLift.initialize()` added to
  `initialize()` — tares each motor and seeds its position reading.
- Added `clawRotationLift.getPosition()` / `clawLiftLift.getPosition()` to the
  existing LCD `screen_task` (lines 6/7), specifically so the real up/down
  travel range of the new tilt mechanism can be read off by jogging the claw by
  hand (or via the still-present B/Y/L1/L2 hold buttons) before calling
  `addPosition()` on it.
- The old hold-voltage buttons (B/Y for `clawLift`, L1/L2 for `clawRotation`)
  are untouched and still drive the motors directly via the separate raw
  `pros::Motor clawLift`/`clawRotation` objects declared earlier in the file.
  This is intentional for now — nothing calls `moveTo`/`hold`/`setOutput` on the
  new `Subsystem` objects yet, so there's no actual conflict — but it means two
  independent motor handles exist on ports 9 and 10 until the hold buttons are
  replaced with macros in a follow-up.

**Next step, not yet done:** tune each PID's gains, find `clawLift`'s real
up/down degrees from the LCD readout, call `addPosition()` on both, then wire
macros to buttons and remove the now-redundant raw hold-button code.

### 6. `clawRotation`: gravity-hold in driver control, and an autonomous tuning routine

Scoped to `clawRotationLift` only (`clawLift`/the tilt mechanism is still untouched,
per point 5). Read `src/liftlib/subsystem.cpp` itself (not just the header) before
writing this, since it settles three things the header alone doesn't make obvious:
`setOutput()` already calls `stopHoldTask()` internally, so jogging and holding
can share one `Subsystem` with no manual bookkeeping; `hold()`/`holdActively()`
silently degrade to a plain `brake()` if no `Feedforward` is configured; and
`holdActively()` tears down and restarts a background task on every call, so it
belongs on an edge, not called every tick.

- **`clawRotationLift.setFeedforward(liftlib::Feedforward::cosine(0.0f, 0.0f, 1.0f))`**
  added in `initialize()`. `clawRotation` pivots (can point at the ceiling or the
  ground), so holding torque varies with angle — `Cosine`, not `Constant`. Both
  `kG` and `horizontal` are untuned placeholders: `kG` needs raising from 0 until
  the claw holds level under its own weight, and `horizontal` assumes the claw
  is level when the program boots (0 on the tared scale) — wrong if that's not
  physically true.
- **Driver control (`opcontrol`)**: L1/L2 now jog `clawRotationLift.setOutput(±CLAW_ROT_JOG_POWER)`
  (untuned, 60.0f) instead of writing `move_voltage` on the raw motor. On the
  release edge (tracked via a `clawRotJogging` bool, not called every idle tick)
  it calls `clawRotationLift.holdActively()` once, so the background hold task
  picks up and fights gravity using the feedforward above. Also called once
  before the main loop starts, so the claw is actively held from boot, not only
  after the first press.
- **The raw `pros::Motor clawRotation` global was removed** — after the above
  change nothing referenced it anymore (verified with a repo-wide grep first).
  The physical motor on port 10 is now driven exclusively through
  `clawRotationLift`'s internal motor handle.
- **`autonomous()` temporarily repurposed as a claw-tuning routine.** The
  previous `chassisAsync(hololib::turnToHeading(90))` + `waitUntilDone()` body is
  commented out, not deleted, and now calls
  `clawRotationLift.moveTo(90.0f, /*async=*/false, 3000)` instead — a blocking
  90-degree move so the LCD's "Claw rot" reading (added last entry) shows where
  it actually stopped once `autonomous()` returns. Sign of the `90.0f` is a
  guess; flip it if the claw swings toward the ceiling instead of the ground.
  **Restore the commented-out lines and remove this block once the PID is tuned**
  — it isn't meant to be the permanent `autonomous()`.

**Still not done:** `kP`/`kD` on `clawRotationPID`, `kG`/`horizontal` on the new
feedforward, and `CLAW_ROT_JOG_POWER` are all placeholder numbers untested on
the robot. `clawLift` (the tilt mechanism) has no feedforward, no gravity-hold,
and is still on its old raw hold-buttons (B/Y) — this entry only covers
`clawRotation`.

### 7. `clawRotationPID` set up for a manual oscillation-hunt tuning pass

`kD` and `kI` zeroed (`0.4f`/`0.02f` → `0.8f`/`0.0f`/`0.0f`) since any damping
would mask the point the user is looking for: run `autonomous()`'s 90-degree
test move, double `kP` each time until it visibly overshoots and oscillates
around the target, then back off to ~50-60% of that value before adding `kD`.
No real torque/inertia numbers exist for this claw (unlike the main lift, which
has `arm_mass_kg`/`kG_base` to derive from), so `0.8f` is a starting point to
double from, not a computed value.

On direction: the user asked whether the 90-degree test move in `autonomous()`
goes up or down. Answered from evidence, not a guess — before the port swap,
this same mechanism (then on port 9, driven by `move_absolute`) rotated from
facing-up toward parallel-then-ground under positive commanded degrees, and
`MotorConfig::gear_ratio` here is positive (`1/12`), so `+90.0f` should still
mean toward the ground, provided the motor wasn't physically flipped when it
was rewired to port 10. Flagged as an inference, not a certainty — first run
should be watched closely.

### 8. Build fix: invalid float literal from manual kP edit

The user bumped `clawRotationPID`'s `kP` from `0.8f` to `3f` while testing — `3f`
isn't a valid C++ float literal (needs a decimal point or exponent: `3.0f`,
`3.f`, or `3e0f`), so `pros build` failed with `unable to find numeric literal
operator 'operator""f'`. Fixed to `3.0f`; the value itself (`3.0`, their tuning
result, not `0.8`) was left as they set it.

### 9. Fixed: the pivot PID was wired to the wrong physical motor

The user corrected the mechanism-to-port mapping used throughout points 5-8:
**port 9 is the vertical-angle pivot** (ceiling-to-ground), **port 10 is the claw
open/close gripper** — the reverse of what those entries assumed. Everything
built as `clawRotationLift` (the tunable PID, `Feedforward::cosine` gravity
hold, the L1/L2 jog logic, and the autonomous 90-degree tuning routine) had been
pointed at port 10 — the gripper — while the actual pivot motor (port 9) was
still on raw, untuned hold-voltage via B/Y. Caught before any of it ran on
hardware.

Fixed by swapping which port each `Subsystem` targets, not by touching any of
the tuning logic itself:
- `clawRotationLift`'s `MotorConfig::port` changed `10 → 9`. Its PID gains,
  feedforward, L1/L2 jog/hold wiring, and the autonomous tuning routine are all
  otherwise unchanged — they were built against the right *behavior*, just the
  wrong *port*.
- The other Subsystem's port changed `9 → 10`, and it was renamed
  `clawLiftLift`/`clawLiftPID` → `clawGripperLift`/`clawGripperPID` to stop
  perpetuating the wrong mental model in the code itself — it's an open/close
  gripper, not a "tilt". Its 2.7:1 ratio and lack of tuning are unchanged; still
  not wired to anything.
- The raw `pros::Motor clawLift` (port 9) — driven by the B/Y hold-buttons —
  was renamed `clawGripper` and moved to **port 10**, since B/Y's actual
  physical target is the gripper, not the pivot. Without this, B/Y's raw
  `move_voltage` calls would have kept firing on port 9 every tick, directly
  fighting `clawRotationLift`'s PID and hold task on the same port — the exact
  dual-motor-object conflict flagged as a temporary, watched tradeoff back in
  point 5, except this time on the wrong motor and with no plan to notice it.
- LCD line 7 relabeled "Claw lift" → "Claw grip" to match.

**None of this has run on the robot yet** — caught from the user's correction
alone, before any upload. The oscillation-hunt tuning from point 8
(`kP = 3.0f`) is now actually pointed at the pivot motor and safe to test.

### 9b. First real hardware run: claw swung ~180 degrees, not 90

User (kP had been lowered to `1.0f` by them since) ran the autonomous test and
the claw visibly moved about twice the commanded 90 degrees. Two different
causes would produce this, needing opposite fixes:
- `gear_ratio` off by 2x (LCD would read ~90 despite ~180 of real travel) — a
  units problem.
- Genuine overshoot/runaway (LCD would read ~180, matching what was seen) — a
  control problem.

No LCD reading was taken on that run, so root cause is still open. Before
re-testing, added a re-tare in `autonomous()` right before the move:

```cpp
clawRotationLift.initialize();  // re-tare: claw's current position becomes 0
clawRotationLift.moveTo(90.0f, /*async=*/false, /*timeout=*/3000);
```

`moveTo()` already calls `Subsystem::reset()` internally, but that only clears
PID controller state (integral, previous error) — checked in
`src/liftlib/subsystem.cpp`, it does not touch the motor encoder. Without this,
repeated test runs on one power cycle measure "90 degrees" from wherever the
claw was left by the previous run or by `opcontrol`'s `holdActively()`, not a
known zero, which could itself explain part of the discrepancy. `initialize()`
is the only thing that re-tares (it's the same call used once at boot).

**Still unresolved:** whether this is the gear-ratio or the overshoot case.
Next run needs the LCD's "Claw rot" (line 6) reading checked immediately after
the claw stops.

### 9c. Diagnosed: overshoot, not gear ratio -- and a real ratio spec applied anyway

User reported the 90-degree test still overshot (visually to ~180), attributed
it to the gear ratio, and gave a real spec: 12:60 (a 5:1 reduction). Separately,
a 45-degree test (run without telling this session first) worked perfectly at
the same `kP`.

That combination rules out a constant gear-ratio error as the cause: a wrong
ratio is wrong by the same proportional factor at every target. It cannot be
correct at 45 and off by 2x at 90 -- the math for that would require the ratio
to be simultaneously ~1/12 and ~1/24, which isn't a thing. "Clean at a small
target, overshoots badly at a larger one" is what a zero `kD` looks like:
bigger targets build more speed before the P-only controller starts backing
off, and momentum carries it further past target than a small move ever builds
enough speed to do. This is the overshoot the oscillation-hunt phase (point 8)
was already watching for, just triggered at 90 degrees rather than something
smaller.

Confirmed with the user that 12:60 is a real, measured gearbox spec (not a
guess from the overshoot), so applied it anyway on its own merits --
`gear_ratio` changed `1.0f/12.0f` (an earlier rough empirical guess) →
`12.0f/60.0f`. Flagged in code and here that this does not fix the overshoot by
itself, and that it changes the system's effective loop gain (0.2 vs ~0.083 is
over 2x larger), so the same `kP = 1.0f` may now behave faster/twitchier than
before -- retuning from here, not assuming the old number still applies.

**Next step:** re-run the 90-degree test under the corrected ratio; if it still
overshoots, that confirms `kD` is what's needed next, per the plan already in
the tuning-phase comment.

### 10. Reversed `clawRotationLift`'s direction

`MotorConfig::port` changed `9 → -9`. PROS treats a negative port as "reversed
motor" at the firmware level (same convention already used for the drivetrain's
left-side ports), which flips `move_voltage`/`move_velocity` writes and position
telemetry together and consistently. Deliberately not done by negating
`gear_ratio` instead — that only affects position/`moveTo` math, not the L1/L2
jog (`setOutput()` bypasses `gear_ratio` and writes raw voltage directly), so it
would have left the jog and the PID-commanded moves disagreeing about which way
is positive. Nothing else needed to change: `autonomous()`'s `moveTo(90.0f, ...)`,
the L1/L2 jog, and the LCD readout are all still correct as written.

---

## 2026-08-21

### 1. Added `CLAUDE.md`

New file at the repo root: build commands, layout, the `config.hpp` global-wiring
contract, the motion-function pattern, the concurrency model, and the conventions
that are easy to get wrong (units, axes, angle handling, gain schedules).

Later in the session it also picked up the two conventions that caused the
drivetrain bugs below: `drive()` takes drive-power units, and left-side inversion
lives inside `drive()`.

### 2. Drivetrain: robot would not move

Four separate problems. **Confirmed fixed on the robot** — the drivetrain drives
and field-centric behaves correctly.

**a. `src/main.cpp` did not compile.** `pros::ADIDigitalOut` has no `extend()`,
`retract()`, or `is_extended()` — those belong to `pros::adi::Pneumatics`. The
brain was therefore running an older binary, which by itself accounted for the
robot not responding.

```
- pros::ADIDigitalOut clawPiston('A', false);
+ pros::adi::Pneumatics clawPiston('A', false);
```

**b. Units mismatch between the motion path and `drive()`.** `Chassis::drive`
fed `move_voltage()` directly, so it read its inputs as millivolts (±12000).
`driveControl` scaled the joystick to millivolts first, but the motion functions
did not — and `MoveParams` defaults `maxTranslationSpeed`/`maxRotationSpeed` to
`127`. Every autonomous motion was therefore clamped to 127 mV, about 1% power.

Fixed by making `drive()` own the conversion: it now takes drive-power units
(±127), normalizes the wheel mix on that scale, then multiplies by
`JOYSTICK_SCALING_FACTOR`. `applyCurve` no longer pre-scales.
(`src/robot/chassis.cpp`)

This is the direction `swingTurn` already assumed with its
`std::min(params.maxRotationSpeed, 127.0f)`.

**c. Left drive ports inverted twice.** Commit `636b35c` moved left-side
inversion into `drive()` (`move_voltage(-v(0))` for FL, `-v(3)` for BL) and
flipped the ports positive in `main.cpp` in the same commit. The working tree had
them negative again, which cancelled the inversion — forward on the stick
produced rotation instead of translation.

```
- int frontLPort = -11;   int backLPort = -20;
+ int frontLPort = 11;    int backLPort = 20;
```

**d. Heading came from the drive encoders when the EKF was off.** With the EKF
disabled and no tracking wheels registered, odometry did
`currentPose.theta += d_theta_wheels`, unlike the tracking-wheel branch which
uses the IMU. On an X-drive that drifts fast, and field-centric driving reads
that heading every tick. Now uses `d_theta_meas` (IMU).
(`src/localization/odometry.cpp`)

`odom.setKalmanFilterEnabled(false)` was also removed from `opcontrol`.

### 3. Driver control: heading-correction bugs found while fixing the above

All in `src/robot/chassis.cpp::driveControl`.

- **`headingOffset` did nothing it was documented to do.** It was subtracted from
  `targetHeading` (biasing the hold target by a constant) instead of being applied
  to the field-centric rotation. It now feeds
  `drive(..., currentHeading - headingOffset)` and `targetHeading` is just the
  current heading. The `main.cpp` call site was changed from `90` to `0`, so
  "forward" is the direction the robot starts in.
- **`correction.correctionOn` was never read.** Setting it `false` had no effect.
  Now honored.
- **Sub-deadzone rotation leaked through.** When the turn stick sat inside the
  deadzone and correction was not active, the raw joystick reading was passed to
  the wheels. Now explicitly zeroed.

### 4. Documentation corrected

- `README.md` — the setup example still showed negative left ports (`-3`, `-4`),
  which is what the broken `main.cpp` config was copied from. Now positive, with
  a note that `drive()` handles left-side inversion.
- `docs/usage_guide.md` — added the drive-power-unit note to the
  `chassis.drive(...)` paragraph.

### 5. Driver control: claw bindings

Compiles and links; **not yet verified on the robot.**

| Button | Action | Motor |
| --- | --- | --- |
| UP | rotate claw 90° at the output, toggle | 9 (`clawRotation`) |
| RIGHT | rotate claw 180° at the output, toggle | 10 (`clawLift`) |
| DOWN | intake in (hold) | 8 (`intake`) |
| R1 / R2 | lift up / down (hold) | 6, -7 (`liftMotors`) |
| B | piston toggle | ADI `'A'` |

Both claw buttons are single-press toggles: `get_digital_new_press` fires only on
the press edge, so holding moves the mechanism exactly once. Each alternates
`0 → far → 0` via `move_absolute`, measured from a `tare_position()` taken when
driver control starts. `move_absolute` is one-shot — the motor's onboard PID holds
the target, so nothing is re-sent per tick.

Both travel amounts are `OUTPUT_DEGREES * GEAR_RATIO`, with the ratios **measured
on the robot rather than taken as nominal**:

| Motor | Commanded | Observed at claw | Derived ratio | Target | Final command |
| --- | --- | --- | --- | --- | --- |
| 9 (UP) | 900 motor° | ~75° | 12 motor°/output° | 90° | **1080 motor°** |
| 10 (RIGHT) | 540 motor° | ~200° | 2.7 motor°/output° | 180° | **486 motor°** |

Travel was revised twice during the session: first UP doubled and RIGHT tripled,
then both recalibrated against the measurements above. Travel angles were then
confirmed correct on the robot, and `CLAW_MOVE_RPM` was raised 100 → 200 (the
green cartridge maximum) because the movement felt sluggish.

### 6. Claw startup position and direction

`clawRotation` (motor 9) now has three named positions instead of a two-state flip,
all absolute from a tare taken in `initialize()`:

| Position | Motor° | Meaning |
| --- | --- | --- |
| `STOWED` | 0 | facing up, keeps the robot within size at startup |
| `CLAW_ROT_PARALLEL` | 1080 | parallel with the ground |
| `CLAW_ROT_GROUND` | 2160 | perpendicular, looking down at the ground |

`initialize()` tares the motor and drops the claw from stowed to parallel
automatically, **after** `chassis.calibrate()` — moving it during `imu.reset()`
would shake the robot while the IMU needs to be held still. Driver control then
toggles PARALLEL ↔ GROUND on UP and never returns to STOWED.

Because every target is absolute from the startup tare, `opcontrol` no longer
re-tares `clawRotation`; doing so mid-travel would have mis-zeroed it. `clawLift`
still tares in `opcontrol` as before.

`CLAW_LIFT_OUTPUT_DEGREES` was negated so the RIGHT arrow's first press turns
clockwise and the second turns back.

### 7. Manual jog for the claw rotation motor

X and Y hold-to-move motor 9 by hand at `CLAW_JOG_RPM` (100), X one way and Y the
other, on top of the UP arrow's preset positions.

The handler sits **after** the UP arrow's, so a held jog wins if both fire on the
same tick, and it only writes to the motor while jogging or on the release edge —
sending `move_velocity(0)` every tick would cancel any in-progress `move_absolute`
from the UP arrow. On release the motor stops and brake mode HOLD keeps it there.

Note the two schemes do not track each other: after jogging by hand, the next UP
press still commands the fixed `PARALLEL` / `GROUND` targets, so the claw snaps
back to a known position rather than toggling relative to where it was jogged to.

---

⚠️ Motor 9's measured 12:1 is far off the nominal 1:5 quoted earlier. If that gap
is mechanical loss (a hard stop, slipping gears, or the position PID giving up
under load) rather than real gearing, commanding more degrees will not scale
linearly and may just stall the motor harder.

Also in this section:

- **Intake reverse removed.** RIGHT used to run the intake outward; the intake now
  only runs inward, on DOWN.
- **`clawLift`'s hold binding removed.** It had a per-tick `move_voltage(0)` that
  would have cancelled each `move_absolute` immediately.
- **Brake modes hoisted out of the loop.** They were device writes being re-sent
  every 20 ms.
- **Per-tick `std::cout` removed from the intake block.**

---

## Open items

- **Autonomous gains need re-tuning.** The schedules in `initialize()` were set
  while the output was effectively capped at ~1% (see 2b). On the corrected scale
  `xSched` kP=15 saturates at about 8.5 inches of error, so every motion will run
  at full speed. First autonomous test should be on blocks or with a reduced
  `{.maxTranslationSpeed = 40}`.
- **Claw rotation directions unverified.** Whether positive degrees is clockwise
  depends on how each motor is mounted. If either goes the wrong way, negate
  `CLAW_ROT_OUTPUT_DEGREES` or `CLAW_LIFT_MOTOR_DEGREES`.
- **`clawPiston` ADI port `'A'` is an unconfirmed placeholder.**
- **`tagOdom` prints `No tags detected!` at 10 Hz**, which floods `pros terminal`
  during debugging. Worth gating behind a flag.
- **`"Lift up"` / `"Lift down"` print every tick** in the R1/R2 block.
- **`src/main.cpp` has a blank line between every line**, apparently from a bad
  formatting pass. Collapsing them would roughly halve the file's length.
