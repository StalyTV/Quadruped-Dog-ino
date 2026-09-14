# Pi to Mega serial protocol

The contract between `Quadruped-Dog-pi` and this repository. Both sides must
agree on every byte here. This file is the canonical copy; the Pi repository
carries a duplicate.

Hardware context is in `design.md` section 2. Frames and sign conventions come
from `kinematics.md` section 1.

## 1. What crosses the link

Only the narrow interface from `design.md`: a twist `(vx, vy, omega)`, a body
pose `(roll, pitch, height)`, a mode enum, and an estop bit. Nothing else. Gait
selection, trajectory generation, IK and calibration all stay on the Mega; the
controller mapping, deadzones and logging all stay on the Pi.

The Pi sends **normalised intent**, not physical units. It has no knowledge of
link lengths, stride limits or maximum speed. Every value is a percentage of a
maximum that is configured on the Mega, next to the geometry it depends on.
Retuning top speed is then a firmware constant, not a change on both sides.

## 2. Transport

    /dev/serial0 on the Pi  <->  Serial1 on the Mega, pins 18 (TX) and 19 (RX)
    115200 baud, 8N1, no flow control

Common ground between the two boards is required, as it is for the servo rails
(`design.md` section 3).

Direction is Pi to Mega only in v1. The Mega must not transmit on this link.
Section 8 reserves a sync word for a future telemetry channel so that the two
directions can never be confused.

## 3. Frame layout

Fixed 12 bytes. No length field, no escaping, no variable-length anything.

    byte  field    type  range       meaning
    ----  -------  ----  ----------  ------------------------------------------
    0     sync0    u8    0xA5        frame start
    1     sync1    u8    0x51        direction and version, see section 8
    2     seq      u8    0..255      increments once per frame, wraps
    3     vx       i8    -100..100   body x velocity, forward positive
    4     vy       i8    -100..100   body y velocity, left positive
    5     omega    i8    -100..100   yaw rate about z, counter-clockwise positive
    6     roll     i8    -100..100   commanded body roll lean
    7     pitch    i8    -100..100   commanded body pitch lean
    8     height   i8    -100..100   body height offset, negative crouches
    9     buttons  u8    bitfield    bit 0 estop, bits 1-7 reserved, zero
    10    mode     u8    0..2        0 idle, 1 crawl, 2 trot
    11    crc      u8                CRC-8 over bytes 2 to 10 inclusive

Signs follow the body frame in `kinematics.md` section 1: x forward, y to the
robot's left, z up. Positive `omega` therefore turns left, so pushing the right
stick right must produce a *negative* omega. This is the sign error most likely
to survive to the first drive test, because it produces a robot that works
perfectly except that it turns the wrong way.

`roll` and `pitch` are right-handed rotations about the body x and y axes, the
same convention `kinematics.md` section 5.4 applies them in. Positive roll
raises the robot's left side; positive pitch drops its nose. Both follow from
the frame rather than from taste, so neither should be flipped in the firmware.
If the D-pad leans the wrong way, fix the mapping on the Pi, which is where
preference belongs.

`height` is an offset from the nominal stance height, not an absolute. Zero
means the stance configured on the Mega, which `kinematics.md` section 4 keeps
at 70 to 85 percent of full reach to stay away from the singularity at full
extension. The Mega clamps the offset to whatever range preserves that margin.

Values outside the stated range are clamped by the receiver, never wrapped.

### Why int8 at one percent resolution

One byte per field needs no endianness agreement and no parsing on an AVR with
no FPU, which matters given the compute budget in `kinematics.md` section 7. A
range of plus or minus 100 stays readable in a hex dump. The resolution loss is
not real: the command is filtered on arrival with a 0.1 to 0.2 second time
constant (section 5.5), which smooths steps far larger than one percent.

## 4. CRC-8

Polynomial 0x07 (x^8 + x^2 + x + 1), init 0x00, MSB first, no reflection, no
final XOR. This is CRC-8/SMBUS. It covers bytes 2 to 10, nine bytes. The sync
bytes are excluded because they are constants and carry no information.

```c
uint8_t crc8(const uint8_t *p, uint8_t n)
{
    uint8_t c = 0x00;
    while (n--) {
        c ^= *p++;
        for (uint8_t i = 0; i < 8; i++)
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}
```

A plain XOR checksum is not adequate here and should not be substituted. It
misses every transposition and any even number of bit flips in the same bit
position. UART corruption is typically a burst, several adjacent bits garbled by
one clock glitch, which is precisely the case CRC-8 catches and XOR does not.

## 5. Framing, and what a dropped byte does

A dropped or inserted byte shifts everything after it. The receiver must be able
to find the true frame boundary again without help.

The rule that makes this work: **after a failed CRC, resume the search at the
byte immediately following the sync pattern that was matched**, not at the end
of the rejected frame. Skipping a whole frame's worth of bytes is the common
implementation mistake. Payload data can contain 0xA5 0x51 by coincidence, and a
receiver that skips ahead on failure can lock onto that false boundary and then
reject every real frame that follows, forever.

The simplest correct implementation is a sliding window. Shift each arriving
byte into a 12-byte register and test the whole register every time. It resyncs
by construction, with no state machine to get wrong.

```c
/* Call once per received byte. Returns true when w holds a valid frame. */
bool frame_push(uint8_t w[12], uint8_t b)
{
    memmove(w, w + 1, 11);
    w[11] = b;
    return w[0] == 0xA5 && w[1] == 0x51 && crc8(w + 2, 9) == w[11];
}
```

At 50 Hz this shifts 600 bytes per second, eleven bytes each. That is nothing
against the transcendental budget in `kinematics.md` section 7, and it buys
framing that cannot get stuck.

The cost of the sliding window is that a corrupted frame is dropped rather than
repaired. That is the correct trade: the next frame arrives 20 ms later, and the
command filter absorbs a single missing update invisibly.

## 6. Timing and failsafe

**The Pi transmits at 50 Hz unconditionally**, including all-zero frames while
the sticks are centred. Continuous transmission is what gives the Mega's timeout
meaning. Without it, silence is ambiguous between a dead link and an operator
holding still, and only one of those should stop the robot.

50 Hz is chosen to match the Mega's fallback control rate. At 100 Hz the Mega
simply uses each command twice, which is harmless because the command is
filtered. Bandwidth is 12 bytes at 50 Hz, 600 B/s against the 11520 B/s the link
carries at 115200 8N1, about five percent. The headroom is deliberate; it leaves
room for the telemetry channel in section 8.

**Mega, on no valid frame for 200 ms:** ramp the commanded velocity to zero
through the normal command filter and hold a stand. Do not freeze the phase
clock. `kinematics.md` section 5.3 explains why: a frozen clock strands whichever
foot is mid-swing in the air.

**Pi, on clean exit or controller disconnect:** send several all-zero frames
with `mode = 0` before closing the port, rather than relying on the timeout. The
timeout is the safety net, not the normal path.

**estop, bit 0 of `buttons`:** latched on the Pi, cleared only by an explicit
operator action. The Mega's response is to ramp to zero, hold the stand, and
ignore the twist until the bit clears. It must not go limp. A limp quadruped
falls over, which is worse than the situation that triggered the estop.

`seq` gives drop detection for free. A gap greater than one means frames were
lost. Counting them is the cheapest diagnostic there is for a marginal link, and
worth logging from the start.

## 7. Mode

    0  idle    hold the stand, ignore the twist
    1  crawl   duty 0.75, offsets 0.00, 0.50, 0.75, 0.25
    2  trot    duty 0.50, offsets 0.00, 0.50, 0.50, 0.00

The Pi sends the desired mode in every frame; it is a level, not an event. The
Mega owns the transition. Per `kinematics.md` section 5.1 a gait change happens
at a cycle boundary, or interpolates duty and offsets over about half a second.
Switching mid-cycle teleports feet.

Start in crawl. It is statically stable and forgiving of everything that is
still wrong at that point in the bring-up.

## 8. Versioning and the reverse channel

`sync1` encodes both direction and version, high nibble and low nibble:

    0x51   Pi to Mega, version 1      (this document)
    0x6v   Mega to Pi, version v      reserved, not implemented
    0x52   Pi to Mega, version 2      next incompatible layout change

Any change to the field layout bumps the low nibble. An old receiver then stops
matching the sync pattern and silently ignores the new frames, which is the
correct failure: refusing to decode is always better than decoding wrongly into
a servo command.

Telemetry from the Mega is undefined in v1 and the Mega must stay silent on this
link. When it is added it takes `0x6v`, so neither side can ever mistake an echo
of its own traffic for a command.

## 9. Open items

- Maximum yaw rate and maximum lean are still unmeasured, and both wait on the
  hip positions `r_i`, since neither means anything for a single leg.

  Maximum body velocity is now measured, at least for one leg walking straight:
  a 90 mm stride at 0.65 Hz, or about **60 mm/s**. The ceiling is the knee
  servo's speed rather than anything geometric, so it will not improve by
  commanding more. See `kinematics.md` section 7.

  The height offset is measured but is **not independent of speed**, which
  section 3 assumes it is. A crouch of 13 mm cuts the achievable stride from 90
  mm to about 22. Either the Mega derates the stride as height moves off
  nominal, or the two have to be negotiated on the Pi side; the protocol as it
  stands lets the operator ask for a combination the leg cannot walk.
- ~~Whether 50 Hz remains adequate once the Mega is confirmed at 100 Hz.~~ The
  Mega is confirmed at 100 Hz, with 2.6 ms used of the 10 ms budget for one leg.
  Arrival jitter is still unmeasured, and the link itself is still unbuilt.
- Telemetry payload, whenever the reverse channel is built.
- Whether the D-pad should command a lean directly or trim one incrementally.
  Section 3 assumes direct: held means leaning. See the note in the Pi
  repository's `controller_read.py`.
