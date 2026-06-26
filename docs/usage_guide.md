# Usage Guide

This is the practical companion to the overview. The front page explains *what*
each system does; this page is about *how* to actually use it on your robot, with
the functions you'll call, code you can copy, and the reasoning behind the knobs
you'll end up turning.

If you're just getting started, read the setup section top to bottom. After that,
treat the rest like a reference, jump to the part you need.

[TOC]

---

## Setup and tuning

Everything starts with one `Chassis` object. You give it the four motors, the
IMU, and a `ChassisConfig` describing the robot, and it owns odometry, PID, and
every movement from there.

```cpp
#include "main.h"

pros::Motor front_left(1, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor front_right(2, pros::E_MOTOR_GEAR_GREEN, true,  pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_left(3, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_right(4, pros::E_MOTOR_GEAR_GREEN, true,  pros::E_MOTOR_ENCODER_DEGREES);

pros::Imu imu(5);

ChassisConfig config {
  .trackWidth       = 12.5,   // left wheel to right wheel, inches
  .drivetrainWidth  = 14.0,
  .drivetrainLength = 14.0,
  .wheelDiameter    = 3.25,   // inches
  .gearRatio        = 1.0,    // motor rotations per wheel rotation
  .kfEnabled        = true    // Kalman filter the raw encoders
};

Chassis chassis(front_left, front_right, back_left, back_right, imu, config);
```

Get the directions right when you build the motors. The two right-side motors
are reversed in the example above, that's normal for an X-drive, but it depends
on how yours is wired. If a movement runs away in the wrong direction, a flipped
motor is the first thing to check.

Then calibrate once at the start of the match, before you try to drive anything:

```cpp
void initialize() {
  chassis.calibrate();   // resets the IMU and zeros the encoders
}
```

`calibrate()` takes a couple of seconds and the robot has to sit still while the
IMU settles. Call it in `initialize()`, not right before a move.

### How the axes are laid out

Before you tune anything, know what the controllers are working on. HoloLib runs
three independent PID controllers:

| Controller | Controls | Error is measured in |
| --- | --- | --- |
| **X** | strafing (left/right) | inches |
| **Y** | forward/back | inches |
| **Theta** | heading | degrees |

So "X gains" tune your sideways movement and "Y gains" tune your forward
movement. On a symmetric X-drive these usually end up close to each other, but
they're separate so you can tune them separately if the robot behaves
differently strafing than it does driving straight.

### Setting gains

You hand each controller its gains with `setXGains`, `setYGains`, and
`setThetaGains`. These don't take a single set of numbers, they take a
*schedule* (more on why in [Gain scheduling](#gain-scheduling)). For now, the
short version: each line is `{error, {kP, kI, kD, kF, slew}}`, and the controller
picks gains based on how far it still has to go.

```cpp
void initialize() {
  chassis.calibrate();

  // {error level, {kP, kI, kD, kF, slew}}
  chassis.setXGains({
      {24.0, {9.0, 0.0, 0.6, 0.0, 8.0}},   // far away: get moving
      {0.0,  {6.5, 0.02, 0.9, 0.0, 4.0}}   // dialed in: settle gently
  });
  chassis.setYGains({
      {24.0, {9.0, 0.0, 0.6, 0.0, 8.0}},
      {0.0,  {6.5, 0.02, 0.9, 0.0, 4.0}}
  });
  chassis.setThetaGains({
      {90.0, {2.8, 0.0,  0.18}},
      {0.0,  {3.2, 0.01, 0.22}}
  });
}
```

If you don't want to deal with scheduling yet, just give each one a single line
with a threshold of `0.0`. That's a plain, fixed PID and it's a perfectly good
place to start.

### How to actually tune the PIDs

There's no shortcut here, you tune by watching the robot and adjusting. The order
that saves the most headaches:

1. **Theta first.** A robot that can't hold its heading will fight every
   translation move, so get turning solid before anything else. Use
   `turnToHeading(90)` as your test.
2. **Then Y (forward), then X (strafe).** Test Y with `moveDistance(24)` and X
   with `strafeDistance(24)`.

For each one, the loop is the same:

- Start with only **kP**. Raise it until the robot reaches the target quickly
  and overshoots just a little, a small bounce is what you want here.
- Add **kD** to take the bounce out. kD fights sudden change, so it damps the
  overshoot. Push it until the move stops cleanly without getting twitchy.
- Only add **kI** if the robot consistently stops *just short* of the target and
  sits there. A little goes a long way, and too much makes it wind up and
  overshoot. Most moves don't need much kI at all.
- **kF** (feed-forward) and **slew** are polish. Slew limits how fast the output
  can jump, which smooths out the start of a move so you're not slamming the
  motors from zero.

The PID controller has a few extra features once you outgrow the basics:
sign-flip reset (dumps the integral when you cross the target so it doesn't
carry stale windup), an integral limit, a windup range, and a filtered
derivative. You usually don't need to touch these, but they're there in
[PID.h](include/PID.h) when you do.

### Movement parameters

Almost every autonomous function takes a `MoveParams` as its last real argument.
This is how you control speed, accuracy, and when a move is allowed to give up:

```cpp
MoveParams params {
  .maxTranslationSpeed = 110.0f,  // cap translation output (0-127)
  .maxRotationSpeed    = 90.0f,   // cap rotation output
  .minSpeed            = 12.0f,   // floor so it doesn't stall crawling in
  .exitRange           = 1.0f,    // "close enough" distance to finish
  .earlyExitRange      = 0.0f,    // bail out this far from the target
  .timeout             = 4000,    // hard stop after this many ms
  .async               = true     // queue it and return immediately
};

chassis.moveToPoint(24, 24, params);
```

A few of these earn their keep:

- **`timeout`** is your safety net. If a move can't settle (stuck on a wall, bad
  tune), it ends here instead of hanging your whole routine. Always set one.
- **`exitRange`** is the accuracy/speed trade. Tighter means more precise but the
  robot spends longer fussing at the end.
- **`earlyExitRange`** lets a move end before it fully settles, handy when you
  want to flow straight into the next move without stopping dead.
- **`async`** is covered in [The motion handler](#the-motion-handler). Leave it
  `true` unless you specifically want the call to block.

You can also set one `MoveParams` as the default for everything with
`setMoveParams(params)` so you're not repeating yourself on every call.

---

## Motion functions

These are the autonomous moves. They all share the same shape: pass a target,
optionally pass a `MoveParams`, and the chassis drives there using the gains you
tuned. By default they're asynchronous, they get queued and the call returns
right away, which is what lets you line several up in a row.

### moveToPoint

Drives to an (x, y) coordinate. By default it also rotates to face the point as
it goes, which keeps the front of the robot pointed where it's headed. Set
`angleCorrection` to `false` if you want to hold your current heading and just
slide over there instead.

```cpp
chassis.moveToPoint(36, 24);                 // drive there, facing the point
chassis.moveToPoint(36, 24, params, false);  // drive there, keep current heading
```

It stops correcting heading in the last couple of inches so the robot isn't
spinning while it tries to settle on the spot.

### moveToPose

Like `moveToPoint`, but you also tell it the exact heading to finish at. It works
the translation and the rotation at the same time, so it arrives in position
*and* pointing the right way, instead of driving there and then turning.

```cpp
chassis.moveToPose(48, 24, 90.0f);   // end at (48, 24), facing 90 degrees
```

![moveToPose lands at the coordinate and the heading together](docs/diagrams/move-to-pose.svg)

Use this when the *next* thing you do depends on your heading, lining up on a
goal, scoring, handing off into a turn-free path.

### moveRelative, moveDistance, strafeDistance

These move relative to where the robot is right now, instead of to an absolute
field coordinate. `moveRelative` takes a forward and a sideways distance at once;
`moveDistance` and `strafeDistance` are just the straight-line shortcuts.

```cpp
chassis.moveDistance(18);          // 18 inches forward
chassis.moveDistance(-12);         // 12 inches back
chassis.strafeDistance(10);        // 10 inches sideways
chassis.moveRelative(18, 10);      // forward and sideways together (diagonal)
```

By default they hold your current heading the whole way (`holdHeading = true`),
so the robot tracks straight instead of drifting off-angle. These are great for
short, predictable adjustments where you don't want to think in field
coordinates.

### turnToHeading and turnToPoint

`turnToHeading` rotates in place to an absolute heading. `turnToPoint` rotates in
place until the front faces a coordinate, it figures out the angle for you.

```cpp
chassis.turnToHeading(180.0f);   // face straight back
chassis.turnToPoint(0, 0);       // face the field origin, wherever you are
```

![turnToPoint rotates in place until the front faces the target](docs/diagrams/turn-to-point.svg)

`turnToPoint` is the one you want for aiming, point at a goal or a target before
you shoot or score, without having to do the trig yourself. Both wait for the
robot to actually stop rotating before they call it done, so you don't fire off
the next move while it's still drifting.

### swingTurn

A swing turn holds one side of the drive still and drives the other side, so the
robot pivots around the locked side instead of spinning around its center. The
turn is tighter and it sweeps the robot through an arc, which is useful when
you're tucked against a wall or working in a corner.

```cpp
chassis.swingTurn(90.0f, Chassis::SwingSide::Left);   // pivot around the left side
chassis.swingTurn(90.0f, Chassis::SwingSide::Right);
```

![A swing turn pivots the robot around one locked side](docs/diagrams/swing-turn.svg)

### curveCircle

Drives a smooth circular arc of a given radius until you hit a target heading.
You can let it pick the shorter direction automatically, or force clockwise /
counter-clockwise.

```cpp
chassis.curveCircle(90.0f, 12.0f);    // arc with a 12-inch radius, end at 90 degrees
chassis.curveCircle(90.0f, 12.0f, params, Chassis::CurveDirection::CW);
```

![curveCircle follows an arc of a set radius to a target heading](docs/diagrams/arc-turn.svg)

A bigger radius is a wider, gentler curve; a smaller radius turns tighter. This
is how you round a corner without stopping to turn.

### followPath

This is the big one. You give it a list of points and it follows the whole path
smoothly, instead of stopping at each point like a chain of `moveToPoint` calls.

It works on a *lookahead*: instead of aiming at the nearest point, it aims at a
point a fixed distance ahead on the path. Picture driving by looking down the
road a bit rather than at your own bumper, that's what keeps the motion smooth
and stops the robot from snapping corner to corner. That lookahead distance is
the second argument, in inches.

![Path following aims at a point a set distance ahead on the path](docs/diagrams/path-follow.svg)

```cpp
std::vector<PathPoint> path = {
  {0,  0,  0},
  {12, 18, 0},
  {30, 24, 0},
  {48, 24, 0}
};

chassis.followPath(path, 8.0f);   // 8-inch lookahead
```

The `headingMode` argument decides where the robot points while it drives the
path:

- **`HeadingMode::FollowPath`** (default), the front follows the direction of
  travel, so the robot "drives like a car" along the curve.
- **`HeadingMode::HoldAngle`**, lock to one heading for the whole path and pass
  the angle in `holdAngleDeg`. The robot keeps facing one way while it traces the
  shape, which an X-drive can do and a tank drive can't.
- **`HeadingMode::CustomAngles`**, use the `theta` you stored in each `PathPoint`,
  so you control the heading point by point.

```cpp
chassis.followPath(path, 8.0f, params, HeadingMode::HoldAngle, 45.0f);
```

Set `reversed = true` (the last argument) to drive the path backwards. A couple
of practical notes: tuning the lookahead matters, too short and it wobbles
trying to hug the line, too long and it cuts corners. Start around 6-10 inches
and adjust. And you need at least two points or it won't run.

You can build paths by hand like above, load them from a file with
`parsePathData(...)`, or design them in the simulator (`tools/sim_auton.py`)
and paste the result in.

### Driver control

For the opcontrol period, `driveControl` maps joystick inputs to the drive. The
last real argument turns on **field-centric** mode, where pushing the stick
"forward" always moves toward the same end of the field no matter which way the
robot is currently facing.

![Field-centric driving keeps "forward" pointed the same way no matter the robot's heading](docs/diagrams/field-centric.svg)

```cpp
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  DriveCurves curves {
    .movement = { .curve_multipler = 1.2f, .deadzone = 5.0f },
    .rotation = { .curve_multipler = 1.5f, .deadzone = 5.0f }
  };

  while (true) {
    chassis.driveControl(
      master.get_analog(ANALOG_LEFT_Y),    // forward / back
      master.get_analog(ANALOG_LEFT_X),    // strafe
      master.get_analog(ANALOG_RIGHT_X),   // rotate
      curves,
      true                                  // field-centric
    );
    pros::delay(10);
  }
}
```

The `DriveCurves` shape the stick response, a deadzone kills drift near center,
and the curve multiplier makes small inputs gentler for fine control while still
letting you floor it. There's also an optional `DriveCorrection` that holds your
heading when you let go of the turn stick, so the robot doesn't slowly rotate off
course while you're just translating.

Field-centric relies on the IMU heading. If you skipped `calibrate()` or the IMU
drifts, "forward" will drift with it. Pass a `headingOffset` if you want to
redefine which way "forward" points (for example, to match your driver's view
from across the field).

If you ever need raw, unfiltered control with no PID in the way, `openLoop`
sends power straight to the wheels.

---

## The motion handler

Every autonomous move runs through a single background queue, the `MotionHandler`
that lives on `chassis.motion`. This is what makes the `async` flag work, and it's
worth understanding because it changes how you write routines.

When you call a move with `async = true`, it gets **added to a queue** and the
call returns immediately. A background task pulls moves off the queue one at a
time and runs them in order. With `async = false`, the call blocks and doesn't
return until that move is finished.

Why this matters: async moves let you do other things while the robot drives, run
an intake, raise a lift, check a sensor, without waiting for the wheels to stop.
You stay in control of timing instead of being stuck in a blocking call.

The functions you'll use to manage it:

```cpp
chassis.moveToPoint(24, 24);     // queued, returns right away
chassis.moveToPose(48, 24, 90);  // queued behind the first one

chassis.waitUntilDone();         // block here until the queue is empty
```

- **`waitUntilDone()`**, wait for everything queued to finish. This is your main
  tool for "do these moves, then continue."
- **`waitUntil(distance)`**, wait until the *current* move has covered a certain
  distance, then keep going. Perfect for firing an action partway through a move:

  ```cpp
  chassis.moveToPoint(48, 0);
  chassis.waitUntil(24);     // halfway-ish
  intake.move(127);          // start the intake mid-drive
  chassis.waitUntilDone();
  ```

- **`cancelMotion()`**, stop the move that's running right now.
- **`cancelAllMotions()`**, stop the current move and wipe the queue. Good for a
  hard reset, say, when a sensor says you've grabbed what you came for.
- **`getDistanceTraveled()`**, how far the current move has gone, in inches (or
  meters if you ask for it).

The handler also exposes `isInMotion()`, `isQueueEmpty()`, and a
`setOnMotionStart()` callback if you want to react to moves starting, useful for
logging or kicking off a mechanism automatically.

Because moves queue, calling five moves in a row doesn't run them all at once,
it lines them up. If you want a move to *replace* what's running instead of
waiting behind it, cancel first.

---

## The Pose EKF

Odometry is the robot knowing where it is. The naive way, count wheel rotations
and add them up, drifts: a wheel slips, a sensor reads noisy, tiny errors stack,
and after a few seconds the robot's idea of its position has wandered off from
reality. The `PoseEKF` (Extended Kalman Filter) fixes that by refusing to trust
any one sensor on its own.

### How it works

The filter tracks three numbers, your X, Y, and heading, and runs two steps over
and over, a few hundred times a second:

1. **Predict.** Using how much the wheels moved this tick, it works out where the
   robot *should* be now. It does this with the real holonomic motion model
   (proper arc integration, not a straight-line guess), so curved motion gets
   estimated correctly.
2. **Correct.** It compares that prediction against what the sensors actually
   read, the IMU for heading, tracking wheels or encoders for position, and
   nudges its estimate toward whichever source it trusts more.

![Each cycle the EKF predicts from motion, then corrects with sensors](docs/diagrams/ekf-loop.svg)

"How much it trusts each source" is the whole game, and it's set by *noise*
values. Process noise is how much you distrust the motion model's prediction;
measurement noise is how much you distrust the sensors. The filter weighs them
against each other automatically every tick.

### Using it

The EKF runs by default. You can turn it off and fall back to plain odometry,
which is worth doing while you tune everything else:

```cpp
chassis.setEKFstate(false);   // raw odometry, no filtering
chassis.setEKFstate(true);    // filtering back on
```

If you have dedicated tracking wheels (unpowered wheels on rotation sensors),
register them, this is the single biggest accuracy upgrade you can give odometry,
since they don't slip the way driven wheels do:

```cpp
TrackingWheelConfig vertical {
  .orientation = TrackingWheelOrientation::VERTICAL,
  // wheel size, sensor port, offsets from center...
};
chassis.addTrackingWheel(vertical);
```

To tune the filter's trust, use `setEKFGains`:

```cpp
//            xProcess  yProcess  thetaProcess  measurement
chassis.setEKFGains(0.001f,  0.001f,   0.003f,       0.0001f);
```

- Raise **process noise** if your estimate lags behind reality or reacts too
  slowly, you're telling the filter to lean more on the sensors.
- Lower **measurement noise** to trust the IMU more (good IMU, clean readings);
  raise it if the heading looks jumpy.

The EKF is the hardest part of the library to tune, and a badly tuned filter is
*worse* than no filter — it'll confidently report the wrong position. If you're
not comfortable with noise values yet, run with `setEKFstate(false)` and come
back to it. Get your movements working on plain odometry first.

---

## Gain scheduling

One PID tune can't be good at everything. Tune it to sprint across the field and
it overshoots on small moves. Tune it to settle gently on small moves and it
crawls across long ones. Gain scheduling is the way out: instead of one set of
gains, you define several, each tied to a range of error, and the controller
blends between them as the robot closes in.

### How the schedule reads

Every entry is `{threshold, {kP, kI, kD, kF, slew}}`. The `threshold` is an error
level, how far you still are from the target, in inches for X/Y or degrees for
theta. The controller looks at the current error and:

- below your smallest threshold, it uses that entry's gains,
- above your largest threshold, it uses that entry's gains,
- anywhere in between, it **linearly interpolates** between the two surrounding
  entries, so the gains slide smoothly instead of snapping.

That interpolation is the important part, there's no jarring handoff where the
robot lurches as gains change. The slew rate scales right along with everything
else.

![Aggressive gains far out blend into gentle gains as the robot settles in](docs/diagrams/gain-schedule.svg)

### Setting it up

```cpp
chassis.setYGains({
    {24.0, {9.0, 0.0,  0.6, 0.0, 8.0}},   // far: push hard, fast slew
    {6.0,  {7.5, 0.0,  0.8, 0.0, 6.0}},   // mid: ease off
    {0.0,  {6.0, 0.02, 1.0, 0.0, 3.0}}    // close: gentle, no overshoot
});
```

You don't have to sort the entries, the scheduler sorts them by threshold for
you. The common setup is aggressive gains far from the target and gentle gains as
you settle, which gives you one move that accelerates hard at the start and
stops clean at the end. You can flip that if you need extra push at low speed to
beat static friction near the target, the mechanism doesn't care which way you
go, it just interpolates whatever you give it.

Start simple. A single entry at threshold `0.0` is a plain fixed PID. Get that
working, then add a second entry once you can clearly see the "too aggressive up
close" or "too lazy far away" problem you're trying to solve. Scheduling is a
refinement, not a starting point.

---

## Driver replay

A good driver run is already a decent autonomous routine, you just have to
capture it. HoloLib records where the robot goes while you drive, then re-drives
that path on its own later.

### Recording

While you drive a practice run, `logReplayData` watches the pose and prints a
coordinate line every time the robot has actually moved (more than half an inch
or a degree, so you're not flooded with duplicates while it sits still):

```cpp
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  chassis.logReplayData(master);   // start logging in the background

  while (true) {
    chassis.driveControl(/* ... your normal driving ... */);
    pros::delay(10);
  }
}
```

Drive your run, then grab the printed `x, y, theta` lines from the terminal.
Those points *are* your path. Call `disableReplayDataLogging()` when you want to
stop.

### Playing it back

Turn the points you captured into a `vector<PathPoint>` and hand them to
`runDriverReplay`. It splits the run into segments wherever you reversed
direction and follows each one, so a back-and-forth run plays back as the same
back-and-forth:

```cpp
void autonomous() {
  std::vector<PathPoint> recorded = {
    {0,   0,   0},
    {14,  6,   12},
    {28,  10,  20},
    // ... the points you captured ...
  };

  chassis.setPose(0, 0, 0);          // start where the recording started
  chassis.runDriverReplay(recorded, 8.0f);   // 8-inch lookahead
}
```

You can also keep paths in a file and load them with `parsePathData(...)` instead
of pasting them inline.

Replay records the robot's *path*, not your button presses. It won't fire your
intake, lift, or anything else — that's on purpose, to keep the log small. Drive
the path with replay, and script your mechanisms separately around it (the
[motion handler](#the-motion-handler)'s `waitUntil` is handy for timing those).
Also make sure `setPose` matches where you started the recording, or the whole
run plays back shifted.

---

## Obstacle avoidance

Sometimes the straight line to your target runs through something you can't drive
over. You tell HoloLib where the obstacles are, turn avoidance on, and your
normal moves bend around them instead of plowing through.

### Setting it up

Obstacles are circles, a center and a radius, in the same field units as your
poses (inches). Drop them in, then switch avoidance on:

```cpp
chassis.addObstacle(36, 36, 6);   // something at (36, 36), 6-inch radius
chassis.addObstacle(48, 12, 5);

chassis.setAvoidanceMode(Chassis::AvoidanceMode::On);
```

With avoidance on, `moveToPoint`, `moveToPose`, `moveRelative`, and `followPath`
all route around the obstacles automatically, you don't call anything different.
For something bigger than a clean circle, stack a few overlapping obstacles to
cover the shape.

### How it steers around things

The live avoidance uses an **artificial potential field**. Think of it as a tug
of war every tick: your target pulls the robot toward it like a magnet, and each
nearby obstacle pushes the robot away. Add those forces up and you get a
direction to drive. HoloLib also adds a sideways "tangential" nudge so the robot
*glides around* an obstacle instead of stalling head-on against it.

![The target pulls, obstacles push, and the robot glides around](docs/diagrams/apf-forces.svg)

You tune that push-and-pull with `setPotentialFieldParams`:

```cpp
//                          ka   kr    influenceRadius
chassis.setPotentialFieldParams(5.0f, 50.0f, 15.0f);
```

- **`ka`**, how strongly the target pulls. Too low and obstacles win and the
  robot stalls; too high and it ignores them and clips through.
- **`kr`**, how strongly obstacles push back. Raise it to give them a wider
  berth.
- **`influenceRadius`**, how close the robot has to get before an obstacle starts
  pushing at all. Outside this distance, obstacles are ignored and the robot just
  drives normally.

If your robot isn't roughly square, give the avoidance math your real footprint
with `setRobotDimensionsAvoidance(width, height)` so the clearances account for
your actual size.

Potential fields are reactive — they're built to keep you from crashing into
something you didn't fully plan for. For a tight, repeatable autonomous routine
you'll usually get cleaner results designing a path that already goes around the
obstacle. Treat avoidance as a safety layer, not a substitute for a good path,
and budget time to tune `kr` and the influence radius before you trust it in a
match.

There's also `detectCollision()`, which flags a sudden impact from an
acceleration spike, handy as a trigger if you want to react to actually bumping
something.
