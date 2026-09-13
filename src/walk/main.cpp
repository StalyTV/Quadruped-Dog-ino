/*
 * One leg, walking.
 *
 * Traces the gait of docs/kinematics.md section 5 on a single leg, with the
 * leg on a stand and the foot in the air. Roadmap step 3.
 *
 * The structure is the loop from section 5.5: a phase clock advanced by dt
 * from micros(), a foot position generated as a continuous function of that
 * phase, inverse kinematics on it every tick, then the joint-to-servo mapping
 * and the calibration table. Smoothness is structural rather than tuned, which
 * is why there are no keyframes and no per-direction animations anywhere.
 *
 * Commands, over serial at 115200:
 *
 *   stand        drive to the neutral stance, ramping over two seconds
 *   walk         start the gait
 *   stop         ramp the stride out and hold the stand
 *   off          release every channel
 *   speed <n>    0 to 100 percent
 *   status       state, angles, pulses, loop cost
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "legmath.h"

/* ------------------------------------------------------------ the robot */

static const LegGeom GEOM = { 42.0f, 80.0f, 80.0f, +1 };
static const float BELT_N = 0.5f;          /* hip pulley over knee pulley */

/* Front right leg, on channels 0 to 2. Measured on the bench; see
   docs/kinematics.md section 8. us_per_rad is signed, so a servo that turns
   the other way is simply a negative slope. */
static const uint8_t CH_ROLL = 0, CH_PITCH = 1, CH_KNEE = 2;
static const ServoCal CAL[3] = {
    { 1650.0f,  646.1f, 1349, 1951 },      /* hip roll, magnitude assumed */
    { 1986.3f,  700.7f, 1100, 1770 },      /* hip pitch, 1770 is a hard stop */
    {  185.3f,  646.1f,  850, 2070 },      /* knee, takes q3 and not t3 */
};

/* Stride and stance are what this leg can actually reach, not what the
   geometry alone would allow. The hip pitch stop at 1770 us caps the thigh at
   -17.7 degrees, which is only 22 degrees forward of neutral against 33
   degrees back, so the usable travel is badly off centre. Moving the neutral
   foot position 12 mm behind the pitch axis re-centres it and buys back most
   of the stride the stop costs: 90 mm instead of 70. */
static const Gait BASE = { 90.0f, 0.50f, 30.0f, 125.0f, -12.0f };

/* The knee servo sets the ceiling. The 2:1 belt that doubled its torque also
   doubled its speed demand, and peak demand is in swing, scaling as
   stride*frequency/(1-duty). At a 90 mm stride that is about 0.7 Hz against a
   loaded 400 deg/s. See docs/kinematics.md section 7. */
static const float MAX_GAIT_HZ = 0.70f;

static const uint16_t PWM_HZ = 100;        /* calibration is specific to this */
static const uint8_t  PCA_ADDR = 0x40;
static const uint32_t STEP_US = 10000;     /* 100 Hz control rate */
static const float    CMD_TAU = 0.15f;     /* command filter, section 5.5 */
static const float    STAND_RAMP_S = 2.0f;
static const float    STAND_SLEW_US = 300.0f;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/* ------------------------------------------------------------------ state */

enum State { OFF, STANDING, HOLDING, WALKING };
static State state = OFF;

static float phi = 0.0f;                   /* gait phase, [0, 1) */
static float stride = 0.0f, stride_cmd = 0.0f;
static float freq = 0.0f, freq_cmd = 0.0f;
static float speed_pct = 60.0f;

static float out_us[3] = { 1650.0f, 1500.0f, 1470.0f };
static bool  driven = false;
static float ramp_left = 0.0f;             /* seconds of slew-limited approach */

static float last_deg[3] = { 0, 0, 0 };
static uint32_t ik_fails = 0, clamp_hits = 0;
static uint16_t step_cost_us = 0, loop_hz = 0;

/* ------------------------------------------------------------------- i2c */

static bool i2c_probe(uint8_t a)
{
    Wire.beginTransmission(a);
    return Wire.endTransmission() == 0;
}

static void write_us(uint8_t c, float us)
{
    const float per_count = 1000000.0f / (4096.0f * (float)PWM_HZ);
    uint32_t n = (uint32_t)(us / per_count + 0.5f);
    if (n > 4095) n = 4095;
    pwm.setPWM(c, 0, n);
}

static void release_all(void)
{
    for (uint8_t c = 0; c < 3; c++) pwm.setPWM(c, 0, 4096);
    driven = false;
    state = OFF;
}

/* ------------------------------------------------------------------- loop */

static float filt(float cur, float target, float dt, float tau)
{
    const float a = dt / (tau + dt);
    return cur + (target - cur) * a;
}

static void control_step(float dt)
{
    /* Filter the command, never the output. Filtering joint angles would only
       add lag to a trajectory that is already smooth. */
    stride = filt(stride, stride_cmd, dt, CMD_TAU);
    freq   = filt(freq,   freq_cmd,   dt, CMD_TAU);

    if (state == WALKING || (state == HOLDING && stride > 0.5f))
        phi = fmodf(phi + freq * dt, 1.0f);

    Gait g = BASE;
    g.stride = stride;

    float s, z;
    foot_path(&g, phi, &s, &z);

    /* Straight ahead, so the heading vector is (1, 0) and the foot stays in
       the leg's own plane at y = l1. */
    float t1, t2, t3;
    if (!leg_ik(&GEOM, s, GEOM.l1, z, &t1, &t2, &t3)) {
        ik_fails++;
        return;                            /* hold the previous pose */
    }

    const float q3 = knee_servo_angle(t2, t3, GEOM.ks, BELT_N);
    bool clamped = false;
    const float want[3] = {
        (float)servo_pulse(&CAL[CH_ROLL],  t1, &clamped),
        (float)servo_pulse(&CAL[CH_PITCH], t2, &clamped),
        (float)servo_pulse(&CAL[CH_KNEE],  q3, &clamped),
    };
    if (clamped) clamp_hits++;

    last_deg[0] = degrees(t1);
    last_deg[1] = degrees(t2);
    last_deg[2] = degrees(t3);

    for (uint8_t c = 0; c < 3; c++) {
        if (ramp_left > 0.0f) {
            /* Slew-limited only while standing up. During the gait the
               trajectory is already continuous and limiting it would just
               distort the foot path. */
            const float step = STAND_SLEW_US * dt;
            const float d = want[c] - out_us[c];
            out_us[c] += (d > step) ? step : (d < -step ? -step : d);
        } else {
            out_us[c] = want[c];
        }
        if (driven) write_us(c, out_us[c]);
    }
    if (ramp_left > 0.0f) ramp_left -= dt;
}

/* ---------------------------------------------------------------- command */

static void set_speed(float pct)
{
    speed_pct = constrain(pct, 0.0f, 100.0f);
    if (state == WALKING) {
        /* Ramp stride and frequency together with commanded speed, as section
           5.3 asks. Both scale, so body speed goes as the square of the
           command and the low end stays controllable. */
        const float k = speed_pct / 100.0f;
        stride_cmd = BASE.stride * k;
        freq_cmd   = MAX_GAIT_HZ * k;
    }
}

static void status(void)
{
    static const char *names[] = { "off", "standing", "holding", "walking" };
    Serial.print(F("state ")); Serial.print(names[state]);
    Serial.print(F("  speed ")); Serial.print(speed_pct, 0); Serial.print('%');
    Serial.print(F("  phi ")); Serial.print(phi, 3);
    Serial.print(F("  stride ")); Serial.print(stride, 1);
    Serial.print(F(" mm  freq ")); Serial.print(freq, 2); Serial.println(F(" Hz"));
    Serial.print(F("  t1 ")); Serial.print(last_deg[0], 1);
    Serial.print(F("  t2 ")); Serial.print(last_deg[1], 1);
    Serial.print(F("  t3 ")); Serial.print(last_deg[2], 1);
    Serial.print(F(" deg   pulses "));
    for (uint8_t c = 0; c < 3; c++) { Serial.print(out_us[c], 0); Serial.print(' '); }
    Serial.println();
    Serial.print(F("  loop ")); Serial.print(loop_hz);
    Serial.print(F(" Hz, step costs ")); Serial.print(step_cost_us);
    Serial.print(F(" us of the ")); Serial.print(STEP_US);
    Serial.println(F(" us budget"));
    Serial.print(F("  ik failures ")); Serial.print(ik_fails);
    Serial.print(F(", limit hits ")); Serial.println(clamp_hits);
}

static void command(char *s)
{
    while (*s == ' ') s++;
    char *arg = strchr(s, ' ');
    if (arg) { *arg++ = 0; while (*arg == ' ') arg++; }

    if (!strcmp(s, "stand")) {
        stride_cmd = 0.0f; freq_cmd = 0.0f;
        stride = 0.0f; freq = 0.0f; phi = 0.0f;
        ramp_left = STAND_RAMP_S;
        driven = true;
        state = STANDING;
        Serial.println(F("standing, two second ramp"));

    } else if (!strcmp(s, "walk")) {
        if (state == OFF) { Serial.println(F("stand first")); return; }
        state = WALKING;
        set_speed(speed_pct);
        Serial.println(F("walking"));

    } else if (!strcmp(s, "stop")) {
        /* Ramp the stride out and let the clock keep running, so whichever
           foot is mid-swing finishes it. Freezing the phase would strand it in
           the air. */
        stride_cmd = 0.0f; freq_cmd = 0.0f;
        state = HOLDING;
        Serial.println(F("stopping, legs finish their swing"));

    } else if (!strcmp(s, "off")) {
        release_all();
        Serial.println(F("released"));

    } else if (!strcmp(s, "speed") && arg) {
        set_speed((float)atof(arg));
        Serial.print(F("speed ")); Serial.print(speed_pct, 0);
        Serial.print(F("%  stride ")); Serial.print(stride_cmd, 0);
        Serial.print(F(" mm  freq ")); Serial.print(freq_cmd, 2);
        Serial.print(F(" Hz  ->  ")); Serial.print(stride_cmd * freq_cmd, 0);
        Serial.println(F(" mm/s of body speed"));

    } else if (!strcmp(s, "status")) {
        status();

    } else if (!strcmp(s, "?") || !strcmp(s, "help")) {
        Serial.println(F("stand / walk / stop / off / speed <0-100> / status"));

    } else if (*s) {
        Serial.print(F("unknown: ")); Serial.println(s);
    }
}

/* -------------------------------------------------------------------- main */

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }

    Wire.begin();
    Serial.println(F("\nQuadrudog, one leg walking"));
    if (!i2c_probe(PCA_ADDR)) {
        Serial.println(F("PCA9685 NOT FOUND at 0x40. SDA is pin 20, SCL pin 21."));
    }

    pwm.begin();
    pwm.setPWMFreq(PWM_HZ);
    release_all();

    Serial.print(F("PWM ")); Serial.print(PWM_HZ);
    Serial.print(F(" Hz, control ")); Serial.print(1000000UL / STEP_US);
    Serial.println(F(" Hz"));
    Serial.print(F("stride ")); Serial.print(BASE.stride, 0);
    Serial.print(F(" mm, stance ")); Serial.print(BASE.stance_h, 0);
    Serial.print(F(" mm, neutral ")); Serial.print(BASE.neutral_s, 0);
    Serial.print(F(" mm, max ")); Serial.print(MAX_GAIT_HZ, 2);
    Serial.println(F(" Hz"));
    Serial.println(F("All channels released. Type stand, then walk."));
}

void loop()
{
    static char buf[40];
    static uint8_t len = 0;
    static uint32_t last_step = 0, last_dt = 0, sec_mark = 0;
    static uint16_t loops = 0;

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') { if (len) { buf[len] = 0; command(buf); len = 0; } }
        else if (c == 8 || c == 127) { if (len) len--; }
        else if (len < sizeof(buf) - 1) buf[len++] = c;
    }

    const uint32_t now = micros();
    if (now - last_step >= STEP_US) {
        const float dt = last_dt ? (now - last_dt) * 1e-6f : (STEP_US * 1e-6f);
        last_dt = now;
        last_step += STEP_US;
        if (now - last_step > STEP_US * 4) last_step = now;   /* fell behind */

        const uint32_t t0 = micros();
        control_step(dt);
        step_cost_us = (uint16_t)(micros() - t0);
        loops++;
    }

    if (millis() - sec_mark >= 1000) { sec_mark = millis(); loop_hz = loops; loops = 0; }
}
