# Design overview

Working title of the write-up: *Open-Source Quadruped Robot: Design, Kinematics,
and Control*.

## 1. What this is

A small quadruped walking robot, roughly in the spirit of Boston Dynamics Spot
but at hobby scale and cost. Four legs, three degrees of freedom each, twelve
servos total, mostly 3D-printed structure.

Goals, in priority order:

1. Walk in any direction under Xbox controller command, turning and translating
   at the same time, at continuously variable speed.
2. Hold a commanded body pose (lean, crouch) while walking, using IMU feedback
   for stabilisation.
3. Provide a documented, reproducible open-source design that someone else could
   rebuild.
4. Stretch goal: replace or augment the analytical inverse kinematics with a
   learned model that absorbs real mechanical error.

Non-goals for the first working version: vision, autonomy, dynamic gaits
(bounding, jumping), and terrain adaptation.

## 2. Control architecture

Two computers, split by timing requirements.

    Xbox controller
          |  Bluetooth
          v
    Raspberry Pi                     high level
      - controller input
      - command shaping
      - telemetry, logging
      - future: vision, learned policies
          |  UART, 115200 or faster, framed binary
          v
    Arduino Mega 2560                real time
      - gait clock and trajectory generation
      - inverse kinematics
      - IMU read and body stabilisation
      - joint to servo mapping
      - safety limits, watchdog
          |
      +---+--------+
      |            |
    PCA9685      MPU6050
      |  I2C        I2C
      v
    12 servos

The Pi never talks to a servo. The Mega never waits on the Pi. If the serial
link goes quiet for more than a timeout (suggest 200 ms), the Mega ramps the
commanded velocity to zero and holds a stand.

The interface between them is deliberately narrow: a twist command
`(vx, vy, omega)` plus a body pose `(roll, pitch, height)` plus a small mode
enum. Everything else stays on one side or the other. Keeping this interface
thin is what makes it possible to later move the gait generator to the Pi, or to
swap in a learned policy, without rewriting the firmware.

### Control mapping

- Left stick: `vx`, `vy`. Magnitude scales speed continuously, so half
  deflection is half speed. Apply a radial deadzone, not per-axis.
- Right stick x: `omega`, turn rate, same continuous scaling.
- D-pad up/down: body pitch lean. D-pad left/right: body roll lean.
- Shoulder triggers: crouch, that is, commanded body height.

All of these combine. There are no separate "walk" and "turn" modes and no
per-direction animations. Section 5 of `kinematics.md` explains why one
generator covers the whole space.

## 3. Components

### Arduino Mega 2560

Chosen for pin count and RAM over an Uno. 16 MHz AVR, no floating point unit,
which is the main constraint on loop rate. Software float trig is expensive;
see the budget in `kinematics.md` section 7.

### PCA9685, 16 channel PWM driver

Twelve of sixteen channels used, four spare. I2C to the Mega on SDA pin 20 and
SCL pin 21.

Channel map:

| Ch | Leg        | Joint     |
|----|------------|-----------|
| 0  | Front left | hip roll  |
| 1  | Front left | hip pitch |
| 2  | Front left | knee      |
| 3  | Front right| hip roll  |
| 4  | Front right| hip pitch |
| 5  | Front right| knee      |
| 6  | Rear left  | hip roll  |
| 7  | Rear left  | hip pitch |
| 8  | Rear left  | knee      |
| 9  | Rear right | hip roll  |
| 10 | Rear right | hip pitch |
| 11 | Rear right | knee      |

Two things about this board are easy to get wrong:

- Logic VCC and servo V+ are separate rails. VCC is the 5 V logic supply from
  the Mega side. V+ is the high-current servo supply from the UBEC. Do not bridge
  them.
- The default output frequency is 50 Hz, which quantises servo position to about
  0.44 degrees per count and adds up to 20 ms of latency. Most digital servos
  accept 200 to 333 Hz. Raise the prescaler to whatever the servo datasheet
  permits. This is the single cheapest improvement to motion smoothness
  available, and it is one line of setup code.

### Servos

Twelve waterproof digital servos, 30 kg-cm class. Unchanged from the original
plan.

Open item: the exact model and its rated pulse width range, maximum PWM
frequency, and stall current must be recorded here before calibration. All three
numbers matter.

A note on torque and power that is often stated backwards: a higher torque
rating does not by itself mean higher current draw under the same load. What it
does mean is a higher ceiling, so the peak and stall currents are larger, and
those peaks are what the power system must survive.

### MPU6050 (GY-521), two units

Three-axis accelerometer plus three-axis gyroscope. Used for body orientation
estimate, feeding roll and pitch correction into the body pose before IK.

Only one is needed on the body. The second is a spare, or can go on a leg for
experiments. If both are used on the same bus, note that the MPU6050 has only
two possible I2C addresses, selected by the AD0 pin.

SoftWire by Steve Marple was considered for software I2C on alternative pins.
There is no need for it if the hardware I2C pins are free, and hardware I2C is
faster and more reliable. Use hardware I2C unless a pin conflict forces
otherwise.

### Power

Current parts: 3S LiPo, 900 mAh, high discharge rate, feeding a Hobbywing UBEC
10A V2 set to 6 V for the servos.

    3S LiPo (11.1 V nominal, 12.6 V full)
        |
        +-------------------+
        |                   |
        v                   v
    UBEC 10A -> 6 V     separate 5 V path
        |                   |
        v                   v
    PCA9685 V+           Arduino Mega
        |
        v
    12 servos

Rules:

- Servo power and logic power are separate supplies with a common ground.
  Grounds must be tied together at a single point, or the PWM signals have no
  reliable reference.
- Do not power the Mega from the 6 V servo rail through an inappropriate pin.
  Give it its own regulated path.
- The switch on the UBEC is an output enable, not a voltage selector. Verify
  against the documentation for your specific UBEC revision before relying on
  this.

**Known problem, unresolved.** The 900 mAh pack is undersized for twelve 30 kg
servos. Twelve servos actively holding a pose can exceed 10 A in aggregate,
which is at or past the UBEC's continuous rating, and 900 mAh at that draw is a
runtime of a few minutes. Proposed fix: a pack in the 2200 to 5000 mAh range
with a high C rating, and two UBECs splitting the load front and rear so neither
runs at its limit. The existing pack remains fine for single-leg bench work.

The hip roll joints are the worst case for continuous current. Because the hip
pitch axis is offset laterally from the roll axis by `l1`, each roll servo
statically holds a moment of roughly (body weight per leg) times `l1` whenever
the robot stands. There is no duty cycle relief on that.

With `l1` now measured at 42 mm, this is less alarming than it looked. The
static holding torque per roll servo is:

| Robot mass | Per leg | Roll servo holds | Fraction of 30 kg-cm |
|---|---|---|---|
| 2.0 kg | 0.50 kg | 2.10 kg-cm | 7 percent |
| 2.5 kg | 0.63 kg | 2.62 kg-cm | 9 percent |
| 3.0 kg | 0.75 kg | 3.15 kg-cm | 11 percent |
| 3.5 kg | 0.88 kg | 3.67 kg-cm | 12 percent |

So the roll joints have a wide torque margin even at the heavy end. This is a
calculation of torque, not of current, and the two are not the same thing: a
servo under a light load still draws current hunting around its setpoint, and
the aggregate idle draw of twelve servos is what the power system has to
survive. The known problem above stands. Measure the actual current before
trusting either number.

## 4. Mechanical design

Each leg: hip roll joint on the body, then a link (`l1`) carrying the hip pitch
axis outboard, then thigh (`l2`), then knee, then shank (`l3`) to the foot.

The hip roll axis runs fore-aft. Rotating it swings the whole leg outward and
inward, like moving a leg sideways away from the body, and tilts the `l1` link
up or down. It does not swing the leg forward or back.

The knee points **rearward**: the thigh runs down and back from the hip, and the
shank down and forward to the foot. This is the Boston Dynamics silhouette
rather than the human one, and it is the `ks = +1` case in `kinematics.md`
sections 3 and 4. It is a property of the frame, so it is a compile-time
constant per leg and never a runtime choice.

Neither of the lower two joints is driven directly on its axis, which keeps the
servo mass close to the body at the cost of a mapping between servo angle and
joint angle:

- **Hip pitch**, through a pushrod linkage. The mapping is nonlinear and needs
  inverting; see `kinematics.md` section 6.
- **Knee**, through a toothed belt from a pulley at the hip pitch axis, with an
  idler tensioning it partway along the thigh. The thigh is the carrier, so
  moving the hip moves the knee even with the knee servo held still. The ratio
  is 2:1 reduction, which doubles the torque at the knee and halves the effect
  of servo resolution error, at the cost of the servo needing twice the joint's
  travel. A belt is linear, so this correction is one multiply and one add
  rather than the geometric inversion the pushrod needs.

Servos are DS3230 PRO, 30 kg-cm class. Both the hip pitch and the knee servo sit
at the hip, which is what keeps the moving mass low.

### Measurements needed

None of the kinematics can be numerically correct until these are taken from
CAD, with all joints at their zero positions, measuring between construction
axes through each joint's rotation centre. Never between mounting faces or screw
holes.

- `l1` = **42 mm**: the perpendicular distance from the hip roll axis out to the
  leg plane, that is, how far outboard the pitch axis is carried. Measured to
  the pitch joint's centre, not between the two axis lines — those intersect
  here, so the distance between them is zero and means nothing. See
  `kinematics.md` section 2, which used to get this wrong.
- `l2` = **80 mm**: hip pitch axis to knee axis.
- `l3` = **80 mm**: knee axis to foot contact point, unloaded. If the foot is a
  ball or hemisphere, measure to the centre of the ball and treat the ground as
  one ball-radius higher. Re-measure under load, since a compliant foot is
  shorter when standing on it; that figure is still outstanding.

Because `l2` and `l3` came out equal, maximum reach is 160 mm, there is no
inner unreachable region, and the nominal stance sits at 125 mm with the thigh
bisecting the knee. The derived stance, joint travel and stride limits are
tabulated in `kinematics.md` section 2.
- Body geometry: the position of each leg's hip roll axis in the body frame.
  These are the `r` vectors used for the turning calculation.
- Linkage geometry: servo pivot position, horn radius, pushrod length, and the
  pushrod attachment point on the thigh, all in leg-plane coordinates.

### Open mechanical questions

1. ~~Does the knee angle change when the thigh sweeps with the knee servo held
   still?~~ **Answered: yes.** The knee is driven by a belt from a pulley at the
   hip pitch axis, with the thigh as the carrier, so the joints are coupled by
   construction. The intent was a 1:1 ratio, which would have let the thigh
   sweep without disturbing the shank's absolute angle. The built ratio is
   **2:1**, the hip pulley having half the teeth of the knee pulley, so the
   shank instead follows the thigh at half rate: 90 degrees of thigh sweep moves
   it 45 degrees. Only a 1:1 pair can hold the shank still, so the original
   intent is not reachable with these pulleys. Because a belt is linear the
   correction is still a single term, far cheaper than the pushrod inversion
   this question anticipated. See `kinematics.md` section 6.
2. Is the hip pitch linkage a parallelogram? If the horn arm and the thigh arm
   are equal length and the pushrod is parallel to the pivot-to-pivot line, the
   mapping is 1:1 plus a constant and needs no inversion at all. Worth designing
   for deliberately if the frame can still be changed.
3. Total mass, and therefore whether the 30 kg servos have adequate margin at
   the worst-case stance. Twelve servos alone is roughly 700 g before frame,
   battery, and electronics. Heavier robot means more holding torque needed,
   which is a loop worth closing before committing to final prints.

## 5. Roadmap

1. Bench: one leg, one PCA9685, existing small battery. Calibrate the servo
   tables. Verify IK by commanding known foot positions and measuring them.
2. Linkage inversion, verified against CAD by comparing commanded servo angle to
   observed joint angle over the full range.
3. Gait generator on one leg, in the air. Check that the foot traces the
   expected path and that joint velocity is continuous at touchdown.
4. Four legs on a stand, feet off the ground. Verify phase offsets and that a
   twist command produces sensible per-leg strides.
5. Power system upgrade. Do not skip this before putting weight on the legs.
6. Standing balance with IMU feedback, no walking.
7. Crawl gait on the ground. Crawl before trot: it is statically stable and
   forgiving.
8. Trot, then the full controller mapping.
9. Only then: the learned IK layer, with the analytical solution as the baseline
   to measure against.

## 6. On the learned inverse kinematics

The original idea was to generate servo angles, move the leg, measure actual
foot position with a camera, collect (angles, position) pairs, and train a model
to invert the mapping.

This is a reasonable research direction because the analytical model cannot
capture 3D-printing tolerance, mechanical flex, servo mounting error, horn
spline quantisation, or link length error. A learned model trained on the real
robot can.

It should come second, not first, for two reasons. Analytical IK is needed as
ground truth to tell whether the learned model is correcting real mechanical
error or fitting noise in the camera measurement. And the per-servo calibration
table described in `kinematics.md` section 8 is already a crude version of the
same correction, so it establishes how much error is actually there before
committing to the heavier approach.

A separate note on the reinforcement-learning path (training a walking policy in
simulation and transferring it): this is viable but is gated on hardware, not
software. RL locomotion policies need joint position and velocity feedback, and
PWM hobby servos report neither. Only the commanded angle is known, not the
actual one, and under load those diverge exactly when it matters. Serial bus
servos that report position and load (Feetech STS3215, LewanSoul LX-16A,
Dynamixel) would change that. Without them, expect a large sim-to-real gap.
