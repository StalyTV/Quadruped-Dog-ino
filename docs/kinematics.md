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
triangle isoceles, so the thigh always bisects the knee: `t2 = t3/2` whenever
the foot is directly below the pitch axis. That identity is worth using on the
bench — set the knee, halve it for the hip.

| Quantity | Value |
|---|---|
| Maximum reach from the pitch axis, `l2 + l3` | 160 mm |
| Usable stance band, 70 to 85 percent of reach | 112 to 136 mm |
| Nominal stance height, 78 percent | 125 mm |
| Joint angles at nominal stance | `t1 = 0`, `t2 = 38.7`, `t3 = 77.5` degrees |
| Body height offset range about nominal | -13 to +11 mm |

**Joint travel**, taken as the union over every stride direction at an 80 mm
stride, 30 mm step height, duty 0.5, from the nominal stance:

| Joint | Range | Travel |
|---|---|---|
| `t1` | -21.5 to +24.6 deg | 46 deg |
| `t2` | +13.0 to +68.7 deg | 56 deg |
| `t3` | +49.7 to +111.5 deg | 62 deg |

No joint needs more than 62 degrees, so a standard 180 degree servo has ample
room. Set `min_us` and `max_us` in section 8 from these ranges plus a margin,
not from the servo's full travel.

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
    fx = l2*sin(t2) + l3*sin(t2 - t3)
    fd = l2*cos(t2) + l3*cos(t2 - t3)     // along the rotated down direction

    x = fx
    y = py - fd*sin(t1)
    z = pz + fd*cos(t1)

Note `t2 - t3` for the shank direction: flexion bends the knee backward relative
to the thigh.

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
    t2 = atan2(x, d) + acos( (l2*l2 + D*D - l3*l3) / (2*l2*D) )

`atan2(x, d)` aims the whole leg at the target; the `acos` term rotates the
thigh off that line by however much the knee bend requires. Flipping that `+` to
a `-` mirrors the knee, which is how front and rear legs get different knee
directions if the frame calls for it. Pick per leg, store it as a constant, and
never let it change at runtime.

### Reference implementation

```c
typedef struct { float l1, l2, l3; } LegGeom;

/* Returns false if the target is outside the workspace.
   Angles are only written on success. */
bool leg_ik(const LegGeom *g, float x, float y, float z,
            int8_t knee_branch, float *t1, float *t2, float *t3)
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
    *t2 = atan2f(x, d) + knee_branch * acosf(ch);
    return true;
}
```

This has been verified against the forward kinematics above over 20000 random
reachable targets; the round trip closes to about 1e-13.

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
    if (leg_ik(&geom, f.x, f.y, f.z, branch[i], &t1, &t2, &t3)) {
        write_joint(i, 0, t1);
        write_joint(i, 1, t2);          /* linkage inversion happens inside */
        write_joint(i, 2, t3);
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

**Unresolved.** If the knee angle changes when the thigh sweeps with the knee
servo held still, the joints are kinematically coupled and a correction term is
needed here, between IK and the inversion. Determine this by sweeping the thigh
in CAD and watching the knee. See `design.md`, open mechanical questions.

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
typedef struct { int16_t zero_us; float us_per_rad; int8_t dir; } ServoCal;
ServoCal cal[12];

int pulse = cal[j].zero_us + cal[j].dir * theta[j] * cal[j].us_per_rad;
pulse = constrain(pulse, cal[j].min_us, cal[j].max_us);   /* always */
```

Procedure: with the leg unloaded, command a known geometric pose, measure the
actual joint angle with a protractor or from a photograph, and adjust `zero_us`
until they agree. Then command a second pose far from the first and adjust
`us_per_rad`. Record the per-joint mechanical travel limits at the same time and
enforce them in software.

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
  recorded.
- Knee/thigh coupling unknown.
- Whether the `l1` link has a fore-aft component that must be handled as a fixed
  translation rather than as `l1`.
- Servo model, pulse range, and maximum PWM frequency not yet recorded.
- Measured cost of a transcendental call on the Mega, which sets the loop rate.
