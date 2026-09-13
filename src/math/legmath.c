#include "legmath.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define TWO_PI 6.28318530717958647692f

void leg_fk(const LegGeom *g, float t1, float t2, float t3,
            float *x, float *y, float *z)
{
    /* Hip pitch axis, carried outboard by l1 and rotated about the leg-frame
       x axis by the abduction angle. */
    const float py = g->l1 * cosf(t1);
    const float pz = g->l1 * sinf(t1);

    /* Two-link planar chain in the leg plane. ks decides which way the shank
       leaves the thigh; this robot is +1, knee rearward. */
    const float sh = t2 + (float)g->ks * t3;
    const float fx = g->l2 * sinf(t2) + g->l3 * sinf(sh);
    const float fd = g->l2 * cosf(t2) + g->l3 * cosf(sh);

    *x = fx;
    *y = py - fd * sinf(t1);
    *z = pz + fd * cosf(t1);
}

bool leg_ik(const LegGeom *g, float x, float y, float z,
            float *t1, float *t2, float *t3)
{
    /* Stage 1, abduction. The hip roll joint does not participate in the reach
       problem; it rotates the plane the other two work in. The leg plane is
       tangent to a circle of radius l1 about the roll axis. */
    const float yz = hypotf(y, z);
    if (yz < g->l1) return false;
    const float d = sqrtf(yz * yz - g->l1 * g->l1);
    const float a1 = atan2f(z, y) - atan2f(d, g->l1);

    /* Stage 2, two-link planar, by the law of cosines twice. */
    const float D = hypotf(x, d);
    if (D > g->l2 + g->l3 || D < fabsf(g->l2 - g->l3)) return false;

    float ck = (g->l2 * g->l2 + g->l3 * g->l3 - D * D) / (2.0f * g->l2 * g->l3);
    float ch = (g->l2 * g->l2 + D * D - g->l3 * g->l3) / (2.0f * g->l2 * D);

    /* Clamp before acos, always. Float error at the workspace boundary makes
       acos see 1.0000001 and return NaN, NaN reaches a pulse width, and a
       garbage pulse width breaks a leg. */
    ck = fminf(1.0f, fmaxf(-1.0f, ck));
    ch = fminf(1.0f, fmaxf(-1.0f, ch));

    *t1 = a1;
    *t3 = (float)M_PI - acosf(ck);
    *t2 = atan2f(x, d) - (float)g->ks * acosf(ch);
    return true;
}

void foot_path(const Gait *gt, float p, float *s, float *z)
{
    const float S = gt->stride;

    if (p < gt->duty) {
        /* Stance. The foot is planted, so in the body frame it slides straight
           back at constant speed. */
        const float t = p / gt->duty;
        *s = gt->neutral_s + S * 0.5f - S * t;
        *z = gt->stance_h;
        return;
    }

    /* Swing. Touchdown and liftoff must happen at zero velocity in the GROUND
       frame, which in the body frame means matching the stance velocity rather
       than zero. A plain cycloid has zero endpoint velocity in whatever frame
       it is written in, so it would be discontinuous here; the S/duty
       coefficient is the correction. See docs/kinematics.md section 5.3. */
    const float t = (p - gt->duty) / (1.0f - gt->duty);
    *s = gt->neutral_s - S * 0.5f + S * t
       - (S / gt->duty) * sinf(TWO_PI * t) / TWO_PI;
    *z = gt->stance_h - gt->step_h * (1.0f - cosf(TWO_PI * t)) * 0.5f;
}

float knee_servo_angle(float t2, float t3, int8_t ks, float belt_n)
{
    /* The belt runs from a pulley at the hip pitch axis to one at the knee,
       with the thigh as the carrier, so the thigh's own motion shows up at the
       knee. Command where the geometry wants the knee, then add back what the
       thigh has already contributed. */
    return t2 + (float)ks * t3 / belt_n;
}

uint16_t servo_pulse(const ServoCal *c, float theta, bool *clamped)
{
    float us = c->zero_us + c->us_per_rad * theta;
    if (us < (float)c->min_us) { us = (float)c->min_us; if (clamped) *clamped = true; }
    else if (us > (float)c->max_us) { us = (float)c->max_us; if (clamped) *clamped = true; }
    return (uint16_t)(us + 0.5f);
}
