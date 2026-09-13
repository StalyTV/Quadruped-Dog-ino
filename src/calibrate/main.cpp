/*
 * Servo calibration bench tool.
 *
 * Finds the per-servo table described in docs/kinematics.md section 8:
 *
 *     pulse_us = zero_us + us_per_rad * theta
 *
 * Jog a servo by pulse width, read the real joint angle off the camera rig,
 * record the pair, repeat across the working range, then fit. The fit uses
 * every point rather than two, and reports its residual, which is the number
 * that says whether a straight line describes the joint at all.
 *
 * That residual matters most for the hip pitch. It runs through a pushrod, so
 * a linear fit is only valid if the linkage is a parallelogram. Small, evenly
 * scattered residuals mean it is. Large residuals with an obvious curve to
 * them mean it is not, and the linkage needs the inversion in section 6.
 *
 * The knee is belt driven off the hip pitch axis at 2:1, so its joint angle
 * depends on the thigh as well as on its own servo. Hold the hip at a known,
 * driven position for the whole knee sweep. Record t3 as the angle between the
 * links, shank minus thigh, not the shank's absolute angle.
 *
 * Nothing here is in a control path, so readability wins over cycle count.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

static const uint8_t NUM_CH = 12;
static const uint8_t MAX_PT = 10;

/* Deliberately narrow to start with. A servo driven past its mechanical stop
   stalls, draws its full stall current and cooks itself, and you will not
   necessarily hear it happen. Widen with "range" once the datasheet or a
   careful manual sweep says what this servo actually accepts. */
static const uint16_t DEFAULT_LO_US = 1000;
static const uint16_t DEFAULT_HI_US = 2000;

/* DS3230 PRO, 180 degree variant: 180 deg over 500 to 2500 us, neutral 1500,
   dead band 3 us. Used only to report positions in degrees, never to command.
   The fitted table is what the firmware uses. */
static const float US_PER_DEG = 2000.0f / 180.0f;

/* Approach every commanded position at this rate rather than jumping. A servo
   asked to cross its range instantly will try, and a loaded leg slamming into
   a stop strips horns. Turn it right down with "creep" when hunting for a
   mechanical limit, so that a joint which stops tracking has not already been
   pushed hard into whatever stopped it. */
static float slew_us_per_s = 300.0f;

static const float SAFE_US = 1500.0f;

/* Default for the breakout with no address jumpers bridged. */
static const uint8_t PCA_ADDR = 0x40;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

struct Point {
    float    angle_deg;   /* joint angle measured off the rig */
    uint16_t us;          /* pulse width commanded at the time */
};

struct Channel {
    Point    pt[MAX_PT];
    uint8_t  n;

    uint16_t lo_us, hi_us;      /* soft travel limits */
    float    cur_us, tgt_us;    /* slewing towards tgt */
    bool     driven;

    bool     fitted;
    float    zero_us, us_per_rad, resid_deg;
};

static Channel ch[NUM_CH];
static uint8_t sel = 0;
static uint16_t pwm_hz = 100;
static unsigned long last_slew;
static bool watching = false;

/* --------------------------------------------------------------------- i2c */

static bool i2c_probe(uint8_t addr)
{
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

static void scan_bus(void)
{
    uint8_t n = 0;
    for (uint8_t a = 1; a < 127; a++) {
        if (!i2c_probe(a)) continue;
        Serial.print(F("  device at 0x"));
        if (a < 16) Serial.print('0');
        Serial.println(a, HEX);
        n++;
    }
    if (!n) Serial.println(F("  nothing answered on the bus"));
}

/* ------------------------------------------------------------------ output */

static float us_per_count(void)
{
    return 1000000.0f / (4096.0f * (float)pwm_hz);
}

static void write_us(uint8_t c, float us)
{
    us = constrain(us, 400.0f, 2600.0f);        /* absolute backstop */
    uint32_t counts = (uint32_t)(us / us_per_count() + 0.5f);
    if (counts > 4095) counts = 4095;
    pwm.setPWM(c, 0, counts);
}

static void release(uint8_t c)
{
    pwm.setPWM(c, 0, 4096);                     /* the full-off bit */
    ch[c].driven = false;
}

/* --------------------------------------------------------------------- fit */

/* Least squares of pulse against angle. Returns false if the points do not
   span enough angle for the slope to mean anything. */
static bool fit_channel(uint8_t c)
{
    Channel &k = ch[c];
    if (k.n < 3) {
        Serial.println(F("need at least 3 points, 8 to 10 is better"));
        return false;
    }

    float sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (uint8_t i = 0; i < k.n; i++) {
        float x = radians(k.pt[i].angle_deg);
        float y = (float)k.pt[i].us;
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    float n = (float)k.n;
    float denom = n * sxx - sx * sx;
    if (fabsf(denom) < 1e-6f) {
        Serial.println(F("all points at the same angle, sweep further"));
        return false;
    }

    k.us_per_rad = (n * sxy - sx * sy) / denom;
    k.zero_us    = (sy - k.us_per_rad * sx) / n;

    /* Residual, expressed in degrees of joint error rather than microseconds,
       because degrees are what you can judge. */
    float worst = 0;
    for (uint8_t i = 0; i < k.n; i++) {
        float x = radians(k.pt[i].angle_deg);
        float e = (float)k.pt[i].us - (k.zero_us + k.us_per_rad * x);
        worst = max(worst, fabsf(e));
    }
    k.resid_deg = (fabsf(k.us_per_rad) > 1e-3f)
                ? degrees(worst / fabsf(k.us_per_rad)) : 0.0f;
    k.fitted = true;

    Serial.print(F("ch ")); Serial.print(c);
    Serial.print(F("  zero_us ")); Serial.print(k.zero_us, 1);
    Serial.print(F("  us_per_rad ")); Serial.print(k.us_per_rad, 1);
    Serial.print(F("  worst residual ")); Serial.print(k.resid_deg, 2);
    Serial.println(F(" deg"));

    if (k.resid_deg > 2.0f) {
        Serial.println(F("  that is large. A straight line may not describe"));
        Serial.println(F("  this joint. Check whether the residuals curve; if"));
        Serial.println(F("  they do, the linkage is not a parallelogram."));
    }
    /* A negative slope simply means the servo turns the other way. That is the
       dir field in the doc's table, folded into the sign where it belongs. */
    return true;
}

static void dump(void)
{
    Serial.println();
    Serial.println(F("/* paste into the firmware. theta in radians. */"));
    Serial.println(F("const ServoCal cal[12] = {"));
    for (uint8_t c = 0; c < NUM_CH; c++) {
        Serial.print(F("  { "));
        Serial.print(ch[c].fitted ? ch[c].zero_us : 0.0f, 1); Serial.print(F("f, "));
        Serial.print(ch[c].fitted ? ch[c].us_per_rad : 0.0f, 1); Serial.print(F("f, "));
        Serial.print(ch[c].lo_us); Serial.print(F(", "));
        Serial.print(ch[c].hi_us); Serial.print(F(" },"));
        if (!ch[c].fitted) Serial.print(F("   /* NOT CALIBRATED */"));
        Serial.println();
    }
    Serial.println(F("};"));
    Serial.println();
}

static void show(void)
{
    Channel &k = ch[sel];
    Serial.print(F("ch ")); Serial.print(sel);
    Serial.print(k.driven ? F("  DRIVEN  ") : F("  off     "));
    Serial.print(k.cur_us, 0); Serial.print(F(" us"));
    Serial.print(F(" = ")); Serial.print((k.cur_us - 1500.0f) / US_PER_DEG, 1);
    Serial.print(F(" deg from neutral"));
    Serial.print(F("  (target ")); Serial.print(k.tgt_us, 0);
    Serial.print(F(", limits ")); Serial.print(k.lo_us);
    Serial.print(F(" to ")); Serial.print(k.hi_us);
    Serial.print(F(")  points ")); Serial.print(k.n);
    if (k.fitted) {
        Serial.print(F("  fit: ")); Serial.print(k.zero_us, 1);
        Serial.print(F(" + ")); Serial.print(k.us_per_rad, 1);
        Serial.print(F("*theta, resid ")); Serial.print(k.resid_deg, 2);
    }
    Serial.println();
}

static void help(void)
{
    Serial.println(F(
        "\nsel <c>      select channel 0-11\n"
        "on / off     drive or release the selected channel\n"
        "stop         release every channel\n"
        "us <n>       move to n microseconds\n"
        "j <n>        jog by n microseconds, negative to go back\n"
        "range <a> <b> set the soft travel limits\n"
        "lo / hi      record the current pulse as a travel limit\n"
        "pt <deg>     record the measured joint angle at this pulse\n"
        "undo         drop the last recorded point\n"
        "clear        drop every point on this channel\n"
        "fit          least squares -> zero_us, us_per_rad, residual\n"
        "list         list this channel's points\n"
        "creep <n>    slew rate in us/s. Drop to ~20 to hunt for a stop.\n"
        "watch        report position while moving, to spot a stalled joint\n"
        "freq <hz>    set the PWM frequency. Invalidates every fit.\n"
        "scan         list what answers on the i2c bus\n"
        "show         state of the selected channel\n"
        "dump         print the whole table as C\n"));
}

/* ----------------------------------------------------------------- command */

static void command(char *s)
{
    while (*s == ' ') s++;
    char *arg = strchr(s, ' ');
    if (arg) { *arg++ = 0; while (*arg == ' ') arg++; }

    Channel &k = ch[sel];

    if (!strcmp(s, "sel") && arg) {
        int c = atoi(arg);
        if (c < 0 || c >= NUM_CH) { Serial.println(F("channel out of range")); return; }
        sel = (uint8_t)c; show();

    } else if (!strcmp(s, "on")) {
        /* Always pick the servo up from a safe middle rather than from
           whatever it was last told, which may be across the range. */
        if (!k.driven) { k.cur_us = SAFE_US; k.tgt_us = SAFE_US; }
        k.driven = true;
        write_us(sel, k.cur_us);
        show();

    } else if (!strcmp(s, "off")) {
        release(sel); show();

    } else if (!strcmp(s, "stop")) {
        for (uint8_t c = 0; c < NUM_CH; c++) release(c);
        Serial.println(F("all channels released"));

    } else if (!strcmp(s, "us") && arg) {
        k.tgt_us = constrain((float)atof(arg), (float)k.lo_us, (float)k.hi_us);
        if (!k.driven) Serial.println(F("channel is off, use \"on\" first"));
        show();

    } else if (!strcmp(s, "j") && arg) {
        k.tgt_us = constrain(k.tgt_us + (float)atof(arg),
                             (float)k.lo_us, (float)k.hi_us);
        if (!k.driven) Serial.println(F("channel is off, use \"on\" first"));
        show();

    } else if (!strcmp(s, "range") && arg) {
        char *b = strchr(arg, ' ');
        if (!b) { Serial.println(F("range <lo> <hi>")); return; }
        *b++ = 0;
        k.lo_us = (uint16_t)atoi(arg);
        k.hi_us = (uint16_t)atoi(b);
        Serial.println(F("limits set. These are the only thing between a bad"));
        Serial.println(F("command and a stalled servo, so keep them tight."));
        show();

    } else if (!strcmp(s, "lo")) {
        k.lo_us = (uint16_t)k.cur_us; show();
    } else if (!strcmp(s, "hi")) {
        k.hi_us = (uint16_t)k.cur_us; show();

    } else if (!strcmp(s, "pt") && arg) {
        if (k.n >= MAX_PT) { Serial.println(F("point buffer full, fit or clear")); return; }
        if (fabsf(k.cur_us - k.tgt_us) > 1.0f) {
            Serial.println(F("still moving, wait for it to settle"));
            return;
        }
        k.pt[k.n].angle_deg = (float)atof(arg);
        k.pt[k.n].us = (uint16_t)k.cur_us;
        k.n++;
        Serial.print(F("recorded ")); Serial.print(k.pt[k.n-1].us);
        Serial.print(F(" us at ")); Serial.print(k.pt[k.n-1].angle_deg, 2);
        Serial.print(F(" deg  (")); Serial.print(k.n);
        Serial.println(F(" points)"));

    } else if (!strcmp(s, "undo")) {
        if (k.n) k.n--;
        Serial.print(k.n); Serial.println(F(" points"));

    } else if (!strcmp(s, "clear")) {
        k.n = 0; k.fitted = false;
        Serial.println(F("points cleared"));

    } else if (!strcmp(s, "list")) {
        for (uint8_t i = 0; i < k.n; i++) {
            Serial.print(F("  ")); Serial.print(k.pt[i].us);
            Serial.print(F(" us  ")); Serial.print(k.pt[i].angle_deg, 2);
            Serial.println(F(" deg"));
        }

    } else if (!strcmp(s, "fit")) {
        fit_channel(sel);

    } else if (!strcmp(s, "freq") && arg) {
        pwm_hz = (uint16_t)atoi(arg);
        pwm.setPWMFreq(pwm_hz);
        for (uint8_t c = 0; c < NUM_CH; c++) ch[c].fitted = false;
        Serial.print(F("PWM now ")); Serial.print(pwm_hz);
        Serial.print(F(" Hz, ")); Serial.print(us_per_count(), 2);
        Serial.println(F(" us per count"));
        Serial.println(F("Every fit is now void. Changing the frequency changes"));
        Serial.println(F("the real pulse width, so the whole table must be"));
        Serial.println(F("measured again at the frequency you intend to run."));

    } else if (!strcmp(s, "creep") && arg) {
        slew_us_per_s = constrain((float)atof(arg), 5.0f, 2000.0f);
        Serial.print(F("slew ")); Serial.print(slew_us_per_s, 0);
        Serial.print(F(" us/s = ")); Serial.print(slew_us_per_s / US_PER_DEG, 1);
        Serial.println(F(" deg/s"));

    } else if (!strcmp(s, "watch")) {
        watching = !watching;
        Serial.println(watching ? F("watch on") : F("watch off"));

    } else if (!strcmp(s, "scan")) {
        Serial.println(F("scanning:"));
        scan_bus();

    } else if (!strcmp(s, "show")) {
        show();
    } else if (!strcmp(s, "dump")) {
        dump();
    } else if (!strcmp(s, "?") || !strcmp(s, "help")) {
        help();
    } else if (*s) {
        Serial.print(F("unknown: ")); Serial.println(s);
    }
}

/* -------------------------------------------------------------------- main */

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }

    for (uint8_t c = 0; c < NUM_CH; c++) {
        ch[c].n = 0;
        ch[c].lo_us = DEFAULT_LO_US;
        ch[c].hi_us = DEFAULT_HI_US;
        ch[c].cur_us = SAFE_US;
        ch[c].tgt_us = SAFE_US;
        ch[c].driven = false;
        ch[c].fitted = false;
    }

    /* Check the bus before driving anything. Without SDA and SCL the library
       fails silently: every write goes nowhere, and the symptom is servos that
       simply never move, which looks like a dozen other faults. */
    Wire.begin();
    if (i2c_probe(PCA_ADDR)) {
        Serial.println(F("PCA9685 found at 0x40."));
    } else {
        Serial.println(F("\nPCA9685 NOT FOUND at 0x40. Nothing will move."));
        Serial.println(F("  SDA goes to Mega pin 20, SCL to pin 21."));
        Serial.println(F("  VCC needs 5 V from the Mega, and the grounds must be"));
        Serial.println(F("  common. V+ is a separate rail for the servos.")); 
        Serial.println(F("Scanning the bus:"));
        scan_bus();
        Serial.println();
    }

    pwm.begin();
    pwm.setPWMFreq(pwm_hz);
    for (uint8_t c = 0; c < NUM_CH; c++) release(c);   /* nothing moves on boot */
    last_slew = micros();

    Serial.println(F("\nQuadrudog servo calibration"));
    Serial.print(F("PWM ")); Serial.print(pwm_hz);
    Serial.print(F(" Hz, ")); Serial.print(us_per_count(), 2);
    Serial.println(F(" us per count"));
    Serial.println(F("All channels released. Nothing moves until you say \"on\"."));
    Serial.println(F("\nDS3230 PRO 180 deg: 500-2500 us, neutral 1500, 11.111 us/deg,"));
    Serial.println(F("dead band 3 us = 0.27 deg. Expected fits, sign depending on"));
    Serial.println(F("how the horn went on:"));
    Serial.println(F("  hip roll  direct        us_per_rad ~  637"));
    Serial.println(F("  hip pitch parallelogram us_per_rad ~  637"));
    Serial.println(F("  knee      2:1 belt      us_per_rad ~ 1273  (against joint t3)"));
    Serial.println(F("\nSoft limits start at 1000-2000 us, which is only 90 deg and is"));
    Serial.println(F("NOT enough for the knee. That one needs about 1150 us of span."));
    Serial.println(F("Find its stops by hand first, then widen with \"range\"."));
    Serial.println(F("\nIdentify each channel before trusting the map: select it, turn"));
    Serial.println(F("it on, jog 30 us, and watch which joint moves."));
    Serial.println(F("\nServos draw amps. They run from V+ on the terminal block, fed by"));
    Serial.println(F("the UBEC, never from the Mega's 5 V. The two grounds must still"));
    Serial.println(F("be tied together or the pulses have no reference."));
    help();
}

void loop()
{
    static char buf[48];
    static uint8_t len = 0;

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (len) { buf[len] = 0; command(buf); len = 0; }
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        }
    }

    unsigned long now = micros();
    float dt = (now - last_slew) * 1e-6f;
    last_slew = now;

    float step = slew_us_per_s * dt;
    for (uint8_t c = 0; c < NUM_CH; c++) {
        if (!ch[c].driven) continue;
        if (ch[c].cur_us < ch[c].tgt_us)
            ch[c].cur_us = min(ch[c].cur_us + step, ch[c].tgt_us);
        else if (ch[c].cur_us > ch[c].tgt_us)
            ch[c].cur_us = max(ch[c].cur_us - step, ch[c].tgt_us);
        write_us(c, ch[c].cur_us);
    }

    /* While something is moving, report where it is. A joint that has met a
       mechanical stop keeps taking pulse changes without moving, and seeing
       the commanded figure run away from the joint is the cheapest stall
       detector available without current sensing. */
    static unsigned long last_report;
    if (watching && millis() - last_report > 250) {
        last_report = millis();
        if (fabsf(ch[sel].cur_us - ch[sel].tgt_us) > 0.5f) show();
    }
}
