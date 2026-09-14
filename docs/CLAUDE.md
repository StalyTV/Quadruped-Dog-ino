# Quadruped robot firmware

Open-source 12-DOF quadruped. Arduino Mega 2560 drives 12 servos through a PCA9685.
A Raspberry Pi handles the Xbox controller and sends a twist command over serial.
Full background in `docs/design.md`, all math in `docs/kinematics.md`, the
serial format in `docs/protocol.md`. Read the kinematics doc before touching
anything in `src/math/`.

## Layout

    src/math/       kinematics, gait path, joint-to-servo mapping. No Arduino
                    headers, so test/ builds it with cc on the host.
    src/walk/       the robot firmware. One leg for now.
    src/calibrate/  bench tool that fits the per-servo table.
    tools/          browser rig that reads joint angles off a webcam.

Each firmware is a PlatformIO environment with its own source filter, so only
one setup()/loop() ever compiles. Adding a second one to a directory breaks the
build, which is how this repository started out.

## Conventions that must not drift

- Body frame: x forward, y to the robot's left, z up. Leg frames: x forward,
  y outward from the body, z down. The sign flip on y and z between the two is
  deliberate; do not "fix" it.
- Angles are radians internally, everywhere. Degrees appear only in serial
  debug output and in calibration tables.
- Leg order is fixed and used as an array index everywhere: 0 FL, 1 FR, 2 RL, 3 RR.
- Joint order within a leg: 0 hip roll (abduction), 1 hip pitch, 2 knee.
- Link names: l1 hip roll axis to hip pitch axis, l2 thigh, l3 shank. All are
  axis-to-axis distances, never part-to-part.
- Knee flexion t3 is measured from full extension, so t3 = 0 is a straight leg.
  t3 is never negative. The knee points rearward (thigh down and back, shank
  down and forward), which is the ks = +1 case, and ks is a per-leg compile-time
  constant that appears in both the forward and the inverse kinematics.
- Positive t1 swings the foot inward, toward the body centreline. The prose in
  kinematics.md used to say outward; it was wrong.

## Pitfalls specific to this robot

- Servo angle is not joint angle. The hip pitch runs through a pushrod, but it
  is a parallelogram, so the mapping is q2 = t2 plus a constant that zero_us
  absorbs: nothing to invert. The knee runs through a 2:1 belt off the hip pitch
  axis, so its servo command depends on t2 as well as t3: q3 = t2 + ks*t3/N.
  Moving the hip moves the knee even with the knee servo held still. See
  `docs/kinematics.md`, section 6.
- Enforce travel limits on the servo angles, after that mapping, never on the
  joint angles. A reachable t3 can still demand an unreachable q3.
- Always clamp acos arguments to [-1, 1] and return a reachability flag. An
  unreachable target produces NaN, NaN becomes a garbage pulse width, and a
  garbage pulse width breaks a leg.
- Never call `delay()` in the control path. The gait clock derives dt from
  `micros()`. Loop jitter shows up directly as visible twitching.
- PCA9685 defaults to 50 Hz and to a 100 kHz bus. This project runs 100 Hz PWM
  and 400 kHz I2C. The PWM rate is the one the calibration table is measured
  against, so changing it invalidates the table. The bus rate is what makes
  twelve servo writes fit in the control budget; see kinematics section 7.
- On boot, interpolate slowly to the standing pose over about 2 seconds. Never
  command a pose change faster than the servos can follow while under load.
- Smooth the command inputs (vx, vy, omega, body pose), never the output joint
  angles. Filtering the outputs just adds lag to the gait.

## Build, test and flash

    pio run                             the default environment, walk
    pio run -e calibrate -t upload      the bench tool instead
    pio device monitor                  115200, local echo on

Host tests, no hardware:

    cc -O2 -I src/math -o /tmp/t test/test_legmath.c src/math/legmath.c -lm && /tmp/t

Never upload without asking. Assume the robot is powered and on a stand. Each
sketch names itself in its boot banner, which is the only reliable way to tell
which one is actually running.

## Working style

- The AVR has no FPU. Prefer float over double, avoid `pow`, and be aware that
  each `sin`, `cos`, `atan2`, `acos` costs real time. Budget in section 7 of the
  kinematics doc.
- Pure math (IK, gait curves, linkage) must be testable on the host without
  hardware. Keep it free of Arduino headers so `test/` can compile it with cc.
  Anything that can break a leg is findable on a laptop, and that is where it
  should be found.
- No emojis in code, comments, commit messages, or documentation.
