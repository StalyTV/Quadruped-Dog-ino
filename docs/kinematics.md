# Kinematics, gait generation, and control

This is the reference for everything between a controller input and a servo
pulse width. Hardware context is in `design.md`.

## 1. Frames and conventions

**Body frame.** Origin at the geometric centre of the body. x forward, y to the
robot's left, z up. Standard right-handed frame, and the one the IMU and the
twist command live in.

**Leg frame.** One per leg, origin at that leg's hip roll axis. x forward,
y outward from the body, z down.

The y and z flip between the two frames is deliberate. It makes the IK
identical for all four legs, because "outward" and "down" mean the same thing on
every leg. The conversion is:

    x_leg =  x_body - hip_x
    y_leg =  side * (y_body - hip_y)      side = +1 for left legs, -1 for right
    z_leg = -(z_body - hip_z)

Every leg then uses the same IK function with the same sign conventions, and the
mirroring is handled once, here, rather than being smeared through the math.

**Joint angles.**

- `t1`, hip roll (abduction). Zero when the leg plane is vertical. Positive is a
  right-handed rotation about the leg-frame x axis, which swings the foot
  **inward**, toward the body centreline. The wording here used to say outward;
  it is not. From section 3, `y = l1*cos(t1) - fd*sin(t1)`, so `dy/dt1 = -fd`
  at `t1 = 0`, which is negative. This sign decides `dir` in the section 8
  calibration table and the sense of the abduction limits, so getting it
  backwards mirrors the leg.
- `t2`, hip pitch. Measured from straight down, positive forward.
- `t3`, knee flexion. Measured from full extension, so `t3 = 0` is a straight
  leg and positive means bent.

All angles are radians. Degrees appear only in debug output and calibration
tables.

**Leg indices.** 0 front left, 1 front right, 2 rear left, 3 rear right. Fixed,
used as an array index everywhere.

## 2. Link parameters

- `l1`: the perpendicular distance from the hip roll axis out to the leg plane,
  which is to say how far outboard the hip pitch axis is carried. Measure it
  from the roll axis to the pitch joint's *centre*, not between the two axis
  lines. On this robot those two lines intersect: the roll axis runs fore-aft,
  the pitch axis runs laterally, and they cross at the leg-frame origin, so the
  line-to-line distance between them is zero and carries no information. This
  entry used to call `l1` the common perpendicular between the axes and to say
  that intersecting axes give `l1 = 0`. Read literally that yields zero, which
  the forward kinematics contradict: `py = l1*cos(t1)` puts the pitch joint on a
  circle of radius `l1` about the roll axis.

  Only the component that is lateral at `t1 = 0` belongs here. A fore-aft offset
  is a genuine fixed translation, subtracted from the foot target once at setup,
  because the roll rotation is about that axis and never moves it. A *vertical*
  offset `h` is not: it rotates with `t1` exactly as the lateral one does. Fold
  it in with `r = hypot(l1, h)` as the link length, which shifts the joint zero
  by `atan2(h, l1)` — a constant that `zero_us` in section 8 absorbs anyway.
- `l2`: hip pitch axis to knee axis (thigh).
- `l3`: knee axis to foot contact point (shank).

A dedicated link carries the pitch axis outboard, so `l1` is not negligible.
That has two consequences worth remembering: the hip rises and falls as the leg
abducts, and each roll servo holds a static moment of roughly (weight per leg)
times `l1` whenever the robot stands.

Also needed, per leg: the hip roll axis position in the body frame,
`r_i = (rx, ry)`. These are the moment arms used for turning.

### Measured values, and what follows from them

Taken from CAD at the zero pose. `l3` is still the unloaded figure; re-measure
it under load before trusting the stance height.

    l1 = 42 mm      l2 = 80 mm      l3 = 80 mm

Everything below is derived from those three numbers and should be recomputed if
any of them changes.

**Equal links simplify two things.** With `l2 == l3` the inner workspace bound
`|l2 - l3|` is zero, so there is no unreachable region near the hip and the
`D < fabs(l2 - l3)` test in section 4 can never fire. It also makes the stance
triangle isoceles, so the thigh always bisects the knee: `t2 = -ks*t3/2` whenever
the foot is directly below the pitch axis. That identity is worth using on the
bench — set the knee, halve it for the hip.

| Quantity | Value |
|---|---|
| Maximum reach from the pitch axis, `l2 + l3` | 160 mm |
| Usable stance band, 70 to 85 percent of reach | 112 to 136 mm |
| Nominal stance height, 78 percent | 125 mm |
| Joint angles at nominal stance, `ks = +1` | `t1 = 0`, `t2 = -38.7`, `t3 = +77.5` degrees |
| Shank absolute angle there | +38.7 degrees, down and forward |
| Body height offset range about nominal | -13 to +11 mm |

**Joint travel**, taken as the union over every stride direction at an 80 mm
stride, 30 mm step height, duty 0.5, from the nominal stance:

| Joint | Range | Travel |
|---|---|---|
| `t1` | -21.5 to +24.6 deg | 46 deg |
| `t2` | -68.6 to -13.0 deg | 56 deg |
| `t3` | +49.7 to +111.5 deg | 62 deg |

No joint needs more than 62 degrees, so a standard 180 degree servo has ample
room. Set `min_us` and `max_us` in section 8 from these ranges plus a margin,
not from the servo's full travel.

These are *joint* angles. The knee servo sees `q3 = t2 + ks*t3/N` instead, and
with `N = 1/2` that is a travel of 103 degrees, not 62. The hip pitch servo sees
whatever the pushrod inversion gives. Limits are enforced on the servo angles,
not on these, and the knee is the joint with the least margin.

**Stride limit.** At the 125 mm nominal stance, straight-line strides stay
inside the 85 percent band up to about 100 mm. Beyond that the foot passes 86
percent at the extremes of the path and heads toward the singularity. A
sideways stride is the worse case, reaching 91 percent at 80 mm, because
abduction costs reach that fore-aft motion does not. Clamp the commanded stride
accordingly, and treat 100 mm as the ceiling until measured otherwise.

The `height` field in `protocol.md` scales to the -13 to +11 mm band above. The
Mega owns that clamp; do not widen it without rechecking the reach margin.

## 3. Forward kinematics

Given `t1, t2, t3`, the foot position in the leg frame:

    // position of the hip pitch axis, rotated by t1 about the leg-frame x axis
    py = l1*cos(t1)
    pz = l1*sin(t1)

    // planar 2-link chain, in the plane containing x and the rotated down axis
    fx = l2*sin(t2) + l3*sin(t2 + ks*t3)
    fd = l2*cos(t2) + l3*cos(t2 + ks*t3)  // along the rotated down direction

    x = fx
    y = py - fd*sin(t1)
    z = pz + fd*cos(t1)

`ks` is the knee direction, `+1` or `-1`, a per-leg constant fixed by how the
frame is assembled. It is not a runtime choice.

- `ks = +1` puts the knee rearward: the thigh runs down and back, the shank down
  and forward. This is the Boston Dynamics silhouette, and it is what this robot
  uses.
- `ks = -1` is the human-like mirror, knee forward.

`t3` stays non-negative either way, so "zero is straight, positive is more bent"
holds for both. Only the direction the shank leaves the thigh changes. Earlier
revisions hardcoded `t2 - t3`, which is the `ks = -1` case.

## 4. Inverse kinematics

The key structural fact is that **the hip roll joint does not participate in the
reach problem**. It rotates the plane that the other two joints work in. So peel
it off first and what remains is a two-link planar problem with a closed form.

### Stage 1: abduction

Project the target onto the y-z plane of the leg frame. The foot lies at radius
`sqrt(y^2 + z^2)` from the roll axis; the pitch axis lies at radius `l1`. The
leg plane is tangent to the `l1` circle, so:

    yz = hypot(y, z)
    if (yz < l1) -> unreachable, the target is inside the abduction circle
    d  = sqrt(yz*yz - l1*l1)              // in-plane reach from the pitch axis
    t1 = atan2(z, y) - atan2(d, l1)

### Stage 2: planar two-link

The target is now `(x, d)` in the leg plane. Two applications of the law of
cosines, one for the angle at the knee and one for the angle at the hip:

    D  = hypot(x, d)
    if (D > l2 + l3 || D < fabs(l2 - l3)) -> unreachable
    t3 = PI - acos( (l2*l2 + l3*l3 - D*D) / (2*l2*l3) )
    t2 = atan2(x, d) - ks*acos( (l2*l2 + D*D - l3*l3) / (2*l2*D) )

`atan2(x, d)` aims the whole leg at the target; the `acos` term rotates the
thigh off that line by however much the knee bend requires. The same `ks` from
section 3 selects the knee direction, and it must be the same constant in both
places.

**A correction worth understanding.** Earlier revisions kept `t3 = PI - acos(..)`
and claimed that flipping the sign in front of the second `acos` mirrors the
knee. It does not. `PI - acos(..)` is non-negative by construction, so flipping
only the `t2` term produces a pose that is not a solution to anything: it misses
the commanded foot position by up to 159 mm on this geometry. Mirroring the knee
requires changing the shank direction in the forward kinematics too, which is
what threading `ks` through both does. With `ks` handled consistently the
position round trip closes to 1.6e-13 mm in both directions.

With `l2 == l3` the stance identity picks up the same sign: `t2 = -ks*t3/2`
whenever the foot hangs directly below the pitch axis.

### Reference implementation

```c
typedef struct { float l1, l2, l3; } LegGeom;

/* Returns false if the target is outside the workspace.
   Angles are only written on success.
   ks is the knee direction, +1 rearward or -1 forward, and must match the
   value used in the forward kinematics. */
bool leg_ik(const LegGeom *g, float x, float y, float z,
            int8_t ks, float *t1, float *t2, float *t3)
{
    const float yz = hypotf(y, z);
    if (yz < g->l1) return false;
    const float d = sqrtf(yz*yz - g->l1*g->l1);
    *t1 = atan2f(z, y) - atan2f(d, g->l1);

    const float D = hypotf(x, d);
    if (D > g->l2 + g->l3 || D < fabsf(g->l2 - g->l3)) return false;

    float ck = (g->l2*g->l2 + g->l3*g->l3 - D*D) / (2.0f * g->l2 * g->l3);
    float ch = (g->l2*g->l2 + D*D - g->l3*g->l3) / (2.0f * g->l2 * D);
    ck = fminf(1.0f, fmaxf(-1.0f, ck));      /* clamp: never let acos see */
    ch = fminf(1.0f, fmaxf(-1.0f, ch));      /* an out-of-range argument  */

    *t3 = (float)M_PI - acosf(ck);
    *t2 = atan2f(x, d) - ks * acosf(ch);
    return true;
}
```

This has been verified against the forward kinematics above over 20000 random
reachable targets in each knee direction; the position round trip closes to
about 1e-13.

Note that it is the *position* round trip that closes. Feeding the returned
angles back through the forward kinematics lands on the commanded foot point to
float precision. The angles themselves occasionally differ from the ones the
target was generated from, in roughly one case in two thousand, all of them
near-folded poses at the edge of the workspace where two genuinely different
joint triples reach the same point. That is the kinematics being ambiguous, not
the solver being wrong.

### Two rules that are not optional

**Clamp, and return the flag.** Without the clamps, floating point error at the
workspace boundary makes `acos` see 1.0000001 and return NaN. NaN propagates to
a pulse width, and a garbage pulse width breaks a leg. The caller must handle
`false` by clipping the target to the workspace or holding the previous valid
pose. It must never write servo commands from a failed solve.

**Stay away from full extension.** As `D` approaches `l2 + l3` the solution
becomes singular: a millimetre of foot motion becomes many degrees of joint
motion, and the servos cannot track it. Keep the nominal stance at roughly 70 to
85 percent of full reach. This is also why real quadrupeds stand with visibly
bent knees.

## 5. Gait generation

Do not think in keyframed animations. Keyframes make smoothness a fight, and
they need a separate animation for every combination of heading, turn rate and
speed, which is combinatorially hopeless. Instead, generate foot positions as a
continuous function of a phase clock, and run IK on them every tick. Smoothness
is then structural rather than something you tune.

### 5.1 Phase clock

A single float `phi` in [0, 1), advanced each tick by `dt * gait_frequency` and
wrapped. Each leg has a fixed offset:

    p_i = fmod(phi + offset_i, 1.0)

| Gait  | Offsets (FL, FR, RL, RR) | Duty | Notes                          |
|-------|--------------------------|------|--------------------------------|
| Crawl | 0.00, 0.50, 0.75, 0.25   | 0.75 | Statically stable, start here  |
| Trot  | 0.00, 0.50, 0.50, 0.00   | 0.50 | Diagonal pairs, faster         |

Within a leg's cycle, `p < duty` is stance and the rest is swing.

Change gait only at a cycle boundary, or interpolate the duty and offsets over
about half a second. Switching mid-cycle teleports feet.

### 5.2 Turning falls out of the twist

This is what removes the need for per-direction animations. Take the command as
a twist `(vx, vy, omega)` in the body frame. Each leg's stance velocity is the
body velocity plus the rotational contribution at that leg's hip position:

    v_ix = vx - omega * ry_i
    v_iy = vy + omega * rx_i

That is `v + omega x r` for rotation about z. The stride **vector** for leg i is

    Sv_i = (v_ix, v_iy) * T_stance          where T_stance = duty / gait_freq

with length `S_i = |Sv_i|` and direction `u_i = Sv_i / S_i`. Walking in any
direction while turning at any rate is one formula. There is no turn animation
and no special case.

Guard `S_i` near zero: when the command is zero, `u_i` is undefined. Hold the
previous direction, or freeze the foot at its neutral point.

### 5.3 The foot path

Everything below is scalar, measured along `u_i` from the leg's neutral foot
position, with `s` forward along `u_i` and `z` downward from the hip. Convert
back with `x = neutral_x + s*u_ix`, `y = neutral_y + s*u_iy`.

**Stance.** The foot is planted, so in the body frame it slides straight back at
constant speed:

    t = p / duty
    s = S/2 - S*t
    z = stance_height

**Swing.** The shape here is not arbitrary. The governing constraint is that
**touchdown and liftoff must happen at zero velocity in the ground frame**. A
foot still moving horizontally when it contacts either scuffs or gets shoved,
and the robot lurches. In the ground frame the foot must arrive stationary,
which in the body frame means the swing curve's endpoint velocity must equal the
*stance* velocity, not zero.

The plain textbook cycloid `s = S(t - sin(2*pi*t)/(2*pi))` has zero endpoint
velocity in whatever frame it is written in, so writing it in the body frame
gives a discontinuity at touchdown. The correction is one coefficient:

    t = (p - duty) / (1 - duty)
    s = -S/2 + S*t - (S/duty) * sin(TWO_PI*t) / TWO_PI
    z = stance_height - step_height * (1 - cos(TWO_PI*t)) / 2

Differentiating: `ds/dt` at `t = 0` is `S(1 - 1/duty)`; divided by the swing
duration `T(1 - duty)` this gives `-S/(T*duty)`, exactly the stance velocity.
Checked numerically as well: at S = 80 mm, duty = 0.5, f = 1.6 Hz, both sides of
both transitions give -256 mm/s horizontal and zero vertical.

A consequence that looks wrong on screen and is correct in reality: at duty 0.5,
the foot moves slightly backward just after liftoff before swinging forward. The
body is moving, so the foot only needs to fall behind before catching up.

**Stopping.** Never stop by freezing the phase clock; that strands a foot in the
air. Ramp `S` toward zero through the command filter and let the legs finish
their swings.

**Speed scheduling.** Ramp gait frequency and step height with commanded speed
rather than holding them fixed. Step height also wants to grow with terrain
roughness.

### 5.4 Body pose

The D-pad and triggers set a desired body attitude `(roll, pitch)` and height.
The IMU-based stabiliser adds its correction to the same variables, so there is
one path, not two.

Compute the foot targets as above in a level ground frame, then transform into
the tilted body frame by the inverse of the desired body pose:

    p_body = R_x(roll)^T * R_y(pitch)^T * (p_ground - [0, 0, dz])

Because it is an inverse rotation, the feet stay where they were on the ground
while the body tilts over them, which is what leaning means. Leaning while
walking then composes automatically with no extra logic.

Then convert body frame to leg frame using the mapping in section 1 and call the
IK.

### 5.5 The loop, assembled

```c
dt = (micros() - last) * 1e-6f;         /* never delay() */
last = micros();

cmd = filter(cmd, target_cmd, dt);      /* smooth the command, not the output */
pose = filter(pose, target_pose + imu_correction, dt);

phi = fmodf(phi + gait_freq * dt, 1.0f);

for (int i = 0; i < 4; i++) {
    float p = fmodf(phi + offset[i], 1.0f);
    Vec3 f = foot_path(p, cmd, i);      /* 5.2 and 5.3 */
    f = apply_body_pose(f, pose);       /* 5.4 */
    f = body_to_leg(f, i);              /* section 1 */
    if (leg_ik(&geom, f.x, f.y, f.z, ks[i], &t1, &t2, &t3)) {
        /* joint angles to servo angles, section 6. The knee needs t2 as
           well as t3, because the belt couples it to the hip. */
        float q2 = hip_servo_angle(t2);
        float q3 = t2 + ks[i] * t3 / BELT_N;
        write_servo(i, 0, t1);          /* hip roll is driven directly */
        write_servo(i, 1, q2);
        write_servo(i, 2, q3);
    }                                    /* on failure: hold previous pose */
}
```

Filter time constant on the command: roughly 0.1 to 0.2 s. Filter the twist and
the body pose. Never filter the joint angles, since that only adds lag to a
trajectory that is already smooth.

## 6. Linkage inversion

The hip pitch joint is driven through a pushrod, so **servo angle is not joint
angle** and IK produces the latter. Something has to sit between them.

**If the linkage is a parallelogram** (horn arm and thigh arm equal length,
pushrod parallel to the line between pivots), the mapping is `servo = joint + k`
and there is nothing to do beyond calibration. Confirm this in CAD before
assuming it.

**Otherwise** the mapping is nonlinear: equal servo steps give unequal joint
steps, with gain varying across the range. Invert it with a circle-circle
intersection, which is cleaner than the Freudenstein equation here because the
pushrod's far end position is already known once IK has given the joint angle.

```c
/* A: servo pivot, fixed, in leg-plane coordinates
   P: pushrod attachment on the thigh, from the joint angle
   a: horn radius,  b: pushrod length
   branch: +1 or -1, the assembly mode, chosen once and never changed */
bool horn_angle(Vec2 A, Vec2 P, float a, float b, int8_t branch, float *phi)
{
    Vec2 v = { P.x - A.x, P.y - A.y };
    float L = hypotf(v.x, v.y);
    if (L > a + b || L < fabsf(a - b)) return false;   /* cannot reach */
    float t = (L*L + a*a - b*b) / (2.0f * L);
    float h = sqrtf(fmaxf(0.0f, a*a - t*t)) * branch;
    Vec2 u = { v.x/L, v.y/L };
    Vec2 n = { -u.y, u.x };
    Vec2 Q = { A.x + u.x*t + n.x*h, A.y + u.y*t + n.y*h };
    *phi = atan2f(Q.y - A.y, Q.x - A.x);
    return true;
}
```

Pick `branch` once by checking which root matches the physical assembly. If it
ever flips at runtime the linkage has passed through a toggle position and the
leg will snap to a mirrored pose, so treat a flip as a fault.

### The knee is belt driven, and it is coupled to the hip

This was listed as unresolved in earlier revisions: does the knee angle change
when the thigh sweeps with the knee servo held still? It does. The knee is
driven by a belt from a pulley at the hip pitch axis, with the thigh acting as
the carrier, so the two joints are kinematically coupled by construction.

The good news is that a belt is **exactly linear**, unlike the pushrod above.
There is no circle-circle intersection, no branch, and no toggle position. The
whole correction is one multiply and one add.

Let `N` be the tooth ratio, driving pulley at the hip over driven pulley at the
knee, and `q3` the knee servo angle measured in the same leg-plane reference as
everything else. In the thigh's rotating frame the belt enforces
`(phi_shank - t2) = N*(q3 - t2)`, so the shank's absolute angle is

    phi_shank = (1 - N)*t2 + N*q3

Setting that equal to the `phi_shank = t2 + ks*t3` of section 3 and solving:

    t3 = ks*N*(q3 - t2)           joint angle from servo angle
    q3 = t2 + ks*t3/N             servo angle from joint angle, the one to use

Read the second line as: command the knee where the geometry wants it, then add
back whatever the thigh's own motion has already contributed.

**What `N = 1` would mean.** Equal pulleys give `q3 = phi_shank` exactly. The
servo would then command the shank's absolute angle and the thigh could sweep
freely underneath without disturbing it. That was the design intent, and it is
worth knowing because it is the case every intuition about this mechanism is
built on.

**The ratio is exactly 2:1.** The pulley at the hip, driven by the knee servo,
has half the teeth of the pulley at the knee, so

    N = 1/2       exactly, being a ratio of tooth counts

A belt ratio is a ratio of two integers, so it is exact and no measurement
uncertainty enters. That matters: the knee error runs about 0.9 degrees per one
percent of error in `N`, and because it scales with `t2` it is systematic rather
than noisy, so an approximate value would never average out.

**The coupling is therefore large, not small.** With `N = 1/2` the shank's
absolute angle follows the thigh at half rate: sweep the thigh 90 degrees with
the knee servo locked and the shank rotates 45 degrees. This is nowhere near the
decoupled behaviour the design intended, and the intent is not reachable with
these pulleys, since only `N = 1` holds the shank still.

Confirmed against two photographs of the real leg taken roughly 90 degrees
apart with the knee servo untouched. Reading the joint centres off them gives a
thigh sweep of 91 degrees and a shank sweep of 42, so `1 - N = 0.46` against the
0.50 that 2:1 predicts, the residual being the error in reading angles off a
photograph. As a check, `q3 = t2 + ks*t3/N` evaluates to 49 and 42 degrees in
the two shots; it should be identical, since the knee servo did not move, and
the spread is the same reading error.

**Consequence for servo travel.** Because the knee is geared down 2:1, the servo
must sweep about twice the joint. Over a full gait cycle in every stride
direction the knee joint covers 62 degrees, and the servo covers 103. That is
still inside a 180 degree servo but it is much less margin than the other two
joints have, and it makes the knee the joint most likely to run out of range.
Check the travel limits on this one first. The compensation is that the same
gearing doubles the torque at the knee and halves the effect of servo
resolution error, both of which the knee needs more than the other joints do.

**Order of operations.** The coupling sits between the IK and the calibration
table, and the travel limits must be enforced *after* it. A `t3` that is
perfectly reachable as a joint angle can demand a `q3` the servo cannot deliver,
because `q3` carries the thigh's contribution as well.

    leg_ik -> t1, t2, t3          pure geometry, no mechanism
    hip pitch:  q2 = horn_angle(t2)    section 6, pushrod, nonlinear
    knee:       q3 = t2 + ks*t3/N      belt, linear
    clamp q2, q3 to the mechanical limits
    pulse = cal[j].zero_us + ...       section 8

Keeping the IK free of all of this is deliberate. The geometry is a property of
the robot's dimensions; the coupling is a property of how it happens to be
driven. Mixing them makes both harder to test, and the IK is the half that can
be checked on the host without hardware.

## 7. Loop rate and compute budget

The Mega has no floating point unit, so every `sin`, `cos`, `atan2`, `acos`
costs real time, on the order of 100 to 200 microseconds each. Do not trust that
estimate; measure it with `micros()` on the actual board early, because it sets
the achievable loop rate.

Per tick the arithmetic is roughly: 4 legs, each needing 2 `atan2` and 2 `acos`
for IK plus an `atan2` for the linkage, so on the order of 20 to 30
transcendental calls plus square roots and the trajectory's `sin` and `cos`.

- 50 Hz gives a 20 ms budget and comfortable headroom.
- 100 Hz gives 10 ms and is probably reachable but wants measuring.
- Beyond that, options are a `sin`/`cos` lookup table, fixed-point math, a fast
  `atan2` approximation, or moving the gait generator to the Pi and leaving the
  Mega to do only IK, servo output and safety.

Target 100 Hz, fall back to 50 Hz. Below 50 Hz the motion starts to look
stepped regardless of how good the trajectory is.

Keep the pure math free of Arduino headers so `test/` can compile it with g++ on
the host. Every formula in this document can be regression-tested without
hardware, and most bugs here are math bugs.

## 8. Servo calibration

IK produces geometric joint angles. Servos need pulse widths, and no two servos
are mounted identically: horn splines quantise the zero position, and print
tolerance shifts it further. Store a per-servo table and calibrate once with the
robot on a stand.

```c
typedef struct { float zero_us, us_per_rad; uint16_t min_us, max_us; } ServoCal;
ServoCal cal[12];

int pulse = (int)(cal[j].zero_us + theta[j] * cal[j].us_per_rad);
pulse = constrain(pulse, cal[j].min_us, cal[j].max_us);   /* always */
```

`us_per_rad` is signed, so a servo that turns the other way is simply a negative
slope. The separate `dir` field earlier revisions carried is redundant, and two
places to express one sign is two places to get it wrong.

### Procedure

Run `pio run -e calibrate -t upload` and drive it over the serial monitor at
115200. Everything starts released, so nothing moves until you say so.

Rather than setting `zero_us` from one pose and `us_per_rad` from a second,
**sweep and fit**: record eight to ten (pulse, measured angle) pairs across the
working range and least-squares a line through them. Two points give a slope
with no way to tell whether a line was the right model; a sweep gives the same
two numbers plus a residual, and the residual is the informative part.

For the hip pitch it is the whole point. That joint runs through a pushrod, so a
linear fit is valid only if the linkage is a parallelogram, which is still an
open question in `design.md`. Small, evenly scattered residuals say it is. Large
residuals with a visible curve say it is not, and section 6's inversion is
needed. The tool prints the worst residual in degrees and warns past two.

Three things specific to this leg:

- Measure the **angle between the links**, not each link's absolute angle.
  `t3` is the shank angle minus the thigh angle. Taking the difference also
  cancels any overall roll in the camera.
- The knee cannot be calibrated on its own. The belt couples it to the hip, so
  hold the hip **driven at a known position** for the whole knee sweep, and put
  its contribution back with `q3 = t2 + ks*t3/N`.
- Recalibrate from scratch if the PWM frequency changes. The frequency sets the
  real pulse width, so every number in the table is specific to it.

Assemble at the nominal stance if the frame allows. It centres each servo's
travel in its range, which matters most at the knee, where 2:1 gearing means
103 degrees of servo sweep. The horn spline still quantises the mounted zero to
about 14.4 degree steps on a 25 tooth spline, so expect to be up to 7 degrees
out however carefully you assemble. That residual is exactly what `zero_us` is
for; it does not need to be fixed mechanically.

Record the per-joint mechanical travel limits at the same time and enforce them
in software. Enforce them on the **servo** angle, after the linkage and belt
mappings, never on the joint angle.

This table is a crude version of the same correction the learned IK model would
provide, so how large the residual error is after calibration is the number that
tells you whether the learned layer is worth building.

## 9. Test plan

Host-side, no hardware needed:

1. IK/FK round trip over random reachable targets. Should close to float
   precision.
2. Workspace boundary: confirm `leg_ik` returns false and writes nothing outside
   reach, and never returns NaN.
3. Trajectory continuity: sample the foot path densely across a full cycle,
   numerically differentiate, and assert that horizontal velocity matches
   stance velocity at both transitions and that vertical velocity is zero there.
4. Loop closure: `foot_path(1 - eps)` equals `foot_path(0)`.
5. Twist mapping: pure `omega` with zero `v` should give four stride vectors
   tangent to circles about the body centre, equal in magnitude for a
   symmetric body.

On hardware:

6. Command known foot positions on one leg, measure with calipers, compare.
   This is the number that quantifies mechanical error, and it is the baseline
   for any later learned model.
7. Sweep the thigh with the knee held, measure knee drift. Answers the coupling
   question in section 6.

## 10. Open items

- ~~Link lengths `l1`, `l2`, `l3` not yet measured from CAD.~~ Measured: 42, 80
  and 80 mm, see section 2. `l3` still needs re-measuring under load, and the
  foot ball radius is not yet recorded.
- Hip positions `r_i` in the body frame not yet recorded. This is now the
  binding one: without it there is no turning, and no way to work out how far
  the body can lean before a leg runs out of reach.
- Linkage geometry (pivot, horn radius, pushrod length, attachment) not yet
  recorded, for the hip pitch pushrod.
- ~~Knee/thigh coupling unknown.~~ Resolved: the knee is belt driven off the hip
  pitch axis, so the joints are coupled, linearly, by the tooth ratio `N`. See
  section 6.
- ~~The exact value of `N`.~~ Resolved: exactly `1/2`, the hip pulley having half
  the teeth of the knee pulley. Confirmed against photographs of the real leg.
- Whether the knee servo's 103 degrees of required travel fits inside its
  mechanical limits once the zero position is chosen. This is the tightest of
  the three joints and the one to check first on the bench.
- Whether the `l1` link has a fore-aft component that must be handled as a fixed
  translation rather than as `l1`.
- Servo model, pulse range, and maximum PWM frequency not yet recorded.
- Measured cost of a transcendental call on the Mega, which sets the loop rate.
