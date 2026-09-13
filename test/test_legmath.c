/*
 * Host tests for the pure math. No hardware, no Arduino.
 *
 *     cc -O2 -I src/math -o /tmp/t test/test_legmath.c src/math/legmath.c -lm && /tmp/t
 *
 * Covers the plan in docs/kinematics.md section 9, items 1 to 4, plus the
 * servo mapping. These are the bugs that break legs, and every one of them is
 * findable on a laptop.
 */

#include "legmath.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static const LegGeom G = { 42.0f, 80.0f, 80.0f, +1 };
static const Gait GT = { 90.0f, 0.50f, 30.0f, 125.0f, -12.0f };

static int failures = 0;
static void check(const char *what, int ok, const char *detail)
{
    printf("  %s %s%s%s\n", ok ? "ok  " : "FAIL", what,
           detail && *detail ? "  " : "", detail ? detail : "");
    if (!ok) failures++;
}

static float frand(float a, float b)
{
    return a + (b - a) * ((float)rand() / (float)RAND_MAX);
}

int main(void)
{
    char buf[160];
    srand(7);

    puts("1. IK and FK round trip over random reachable targets");
    {
        float worst_pos = 0.0f;
        int solved = 0, tried = 0;
        for (int i = 0; i < 20000; i++) {
            float t1 = frand(-0.45f, 0.45f);
            float t2 = frand(-1.30f, -0.20f);
            float t3 = frand(0.60f, 1.90f);
            float x, y, z, a1, a2, a3, X, Y, Z;
            leg_fk(&G, t1, t2, t3, &x, &y, &z);
            tried++;
            if (!leg_ik(&G, x, y, z, &a1, &a2, &a3)) continue;
            solved++;
            leg_fk(&G, a1, a2, a3, &X, &Y, &Z);
            float e = fmaxf(fabsf(X - x), fmaxf(fabsf(Y - y), fabsf(Z - z)));
            if (e > worst_pos) worst_pos = e;
        }
        snprintf(buf, sizeof buf, "%d/%d solved, worst position error %.2e mm",
                 solved, tried, worst_pos);
        check("position round trip closes", worst_pos < 1e-3f, buf);
    }

    puts("2. Workspace boundary");
    {
        float a1, a2, a3;
        int nan_seen = 0, wrote_on_fail = 0;
        for (int i = 0; i < 40000; i++) {
            float x = frand(-400.0f, 400.0f);
            float y = frand(-400.0f, 400.0f);
            float z = frand(-400.0f, 400.0f);
            a1 = a2 = a3 = -12345.0f;
            if (leg_ik(&G, x, y, z, &a1, &a2, &a3)) {
                if (isnan(a1) || isnan(a2) || isnan(a3)) nan_seen++;
            } else {
                if (a1 != -12345.0f || a2 != -12345.0f || a3 != -12345.0f)
                    wrote_on_fail++;
            }
        }
        snprintf(buf, sizeof buf, "%d NaNs over 40000 targets", nan_seen);
        check("never returns NaN on success", nan_seen == 0, buf);
        check("writes nothing when it fails", wrote_on_fail == 0, "");

        check("inside the abduction circle is unreachable",
              !leg_ik(&G, 0.0f, 10.0f, 10.0f, &a1, &a2, &a3), "yz < l1");
        check("beyond full extension is unreachable",
              !leg_ik(&G, 0.0f, G.l1, 200.0f, &a1, &a2, &a3), "D > l2+l3");
    }

    puts("3. Foot path continuity at the two transitions");
    {
        const float e = 1e-4f;
        float s0, z0, s1, z1;

        /* Stance velocity, which the swing endpoints must match. */
        foot_path(&GT, 0.10f, &s0, &z0);
        foot_path(&GT, 0.10f + e, &s1, &z1);
        const float v_stance = (s1 - s0) / e;

        foot_path(&GT, GT.duty + e, &s1, &z1);
        foot_path(&GT, GT.duty, &s0, &z0);
        const float v_lift = (s1 - s0) / e;
        const float vz_lift = (z1 - z0) / e;

        foot_path(&GT, 1.0f - e, &s0, &z0);
        foot_path(&GT, 1.0f - 2.0f * e, &s1, &z1);
        const float v_touch = (s0 - s1) / e;
        const float vz_touch = (z0 - z1) / e;

        snprintf(buf, sizeof buf, "stance %.1f, liftoff %.1f mm/cycle", v_stance, v_lift);
        check("horizontal velocity matches at liftoff",
              fabsf(v_lift - v_stance) < 0.02f * fabsf(v_stance), buf);
        snprintf(buf, sizeof buf, "stance %.1f, touchdown %.1f mm/cycle", v_stance, v_touch);
        check("horizontal velocity matches at touchdown",
              fabsf(v_touch - v_stance) < 0.02f * fabsf(v_stance), buf);
        snprintf(buf, sizeof buf, "%.3f and %.3f mm/cycle", vz_lift, vz_touch);
        check("vertical velocity is zero at both transitions",
              fabsf(vz_lift) < 1.0f && fabsf(vz_touch) < 1.0f, buf);
    }

    puts("4. Loop closure");
    {
        float s0, z0, s1, z1;
        foot_path(&GT, 0.0f, &s0, &z0);
        foot_path(&GT, 0.999999f, &s1, &z1);
        snprintf(buf, sizeof buf, "ds %.4f, dz %.4f mm", fabsf(s1 - s0), fabsf(z1 - z0));
        check("foot_path(1-eps) meets foot_path(0)",
              fabsf(s1 - s0) < 0.01f && fabsf(z1 - z0) < 0.01f, buf);
    }

    puts("5. The whole cycle stays reachable and inside the servo limits");
    {
        const ServoCal cal[3] = {
            { 1650.0f,  646.1f, 1349, 1951 },
            { 1986.3f,  700.7f, 1100, 1770 },
            {  185.3f,  646.1f,  850, 2070 },
        };
        int unreachable = 0, clamped_count = 0;
        float peak_reach = 0.0f;
        for (int i = 0; i <= 2000; i++) {
            float s, z, t1, t2, t3;
            foot_path(&GT, (float)i / 2001.0f, &s, &z);
            if (!leg_ik(&G, s, G.l1, z, &t1, &t2, &t3)) { unreachable++; continue; }

            const float d = sqrtf((G.l1 * G.l1 + z * z) - G.l1 * G.l1);
            const float reach = hypotf(s, d);
            if (reach > peak_reach) peak_reach = reach;

            const float q3 = knee_servo_angle(t2, t3, G.ks, 0.5f);
            bool c = false;
            servo_pulse(&cal[0], t1, &c);
            servo_pulse(&cal[1], t2, &c);
            servo_pulse(&cal[2], q3, &c);
            if (c) clamped_count++;
        }
        snprintf(buf, sizeof buf, "%d unreachable samples", unreachable);
        check("every phase of the cycle solves", unreachable == 0, buf);
        snprintf(buf, sizeof buf, "%d samples hit a limit", clamped_count);
        check("no servo limit is hit during the cycle", clamped_count == 0, buf);
        snprintf(buf, sizeof buf, "peak %.1f mm of %.0f, %.0f%% of full extension",
                 peak_reach, G.l2 + G.l3, 100.0f * peak_reach / (G.l2 + G.l3));
        check("stays clear of the singularity",
              peak_reach < 0.90f * (G.l2 + G.l3), buf);
    }

    puts("6. Servo mapping");
    {
        const ServoCal c = { 1986.3f, 700.7f, 1100, 1770 };
        bool cl = false;
        snprintf(buf, sizeof buf, "%u us at the measured neutral",
                 servo_pulse(&c, -0.69402f, &cl));
        check("hip pitch neutral lands back on 1500",
              abs((int)servo_pulse(&c, -0.69402f, &cl) - 1500) <= 1, buf);
        cl = false;
        servo_pulse(&c, -3.0f, &cl);
        check("the clamp flag is raised when the limit bites", cl, "");
    }

    printf("\n%s\n", failures ? "FAILURES ABOVE" : "all checks passed");
    return failures ? 1 : 0;
}
