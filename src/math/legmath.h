/*
 * Leg kinematics, gait trajectory and the joint-to-servo mapping.
 *
 * Free of Arduino headers on purpose, so test/ compiles it with g++ on a
 * laptop. Everything here is a property of the robot's dimensions and of the
 * mechanism, and all of it can be checked without hardware. See
 * docs/kinematics.md sections 3 to 6.
 *
 * Angles are radians everywhere. Lengths are millimetres.
 */

#ifndef LEGMATH_H
#define LEGMATH_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float l1;      /* hip roll axis out to the leg plane */
    float l2;      /* hip pitch axis to knee axis */
    float l3;      /* knee axis to foot contact point */
    int8_t ks;     /* knee direction, +1 rearward (this robot), -1 forward */
} LegGeom;

typedef struct {
    float stride;      /* peak to peak travel of the foot along the heading */
    float duty;        /* fraction of the cycle spent in stance */
    float step_h;      /* how far the foot lifts during swing */
    float stance_h;    /* foot depth below the hip pitch axis at stance */
    float neutral_s;   /* fore-aft offset of the neutral foot position */
} Gait;

typedef struct {
    float zero_us;     /* pulse at which this servo's angle is zero */
    float us_per_rad;  /* signed: negative simply means it turns the other way */
    uint16_t min_us;   /* mechanical travel limits, enforced on the servo angle */
    uint16_t max_us;
} ServoCal;

/* Foot position in the leg frame. Always succeeds. */
void leg_fk(const LegGeom *g, float t1, float t2, float t3,
            float *x, float *y, float *z);

/* Joint angles for a foot position. Returns false and writes nothing if the
   target is outside the workspace. A caller must handle false by holding the
   previous pose: a failed solve must never reach a servo. */
bool leg_ik(const LegGeom *g, float x, float y, float z,
            float *t1, float *t2, float *t3);

/* Foot path for phase p in [0, 1), as a scalar distance along the heading and
   a depth below the hip. docs/kinematics.md section 5.3. */
void foot_path(const Gait *gt, float p, float *s, float *z);

/* Knee servo angle. The belt couples the knee to the hip, so this needs the
   thigh angle as well: q3 = t2 + ks*t3/N. */
float knee_servo_angle(float t2, float t3, int8_t ks, float belt_n);

/* Servo angle to pulse width, clamped to the mechanical limits. Sets *clamped
   when the limit actually bit, which the caller should treat as a fault rather
   than as normal operation. */
uint16_t servo_pulse(const ServoCal *c, float theta, bool *clamped);

#ifdef __cplusplus
}
#endif

#endif
