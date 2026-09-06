# DiscoX — reject bad rangefinder readings

> **CORRECTION (2026-09-06, verified on hardware):** the "SMALLER = BETTER"
> claim below is **wrong**, as is the manual it came from. Bench testing shows
> SQ is a return-amplitude value: **HIGHER = stronger signal**. A white surface
> up close reads a few hundred; a black/specular surface reads 4–6. The
> implemented gate therefore rejects shots with SQ *below* `laser_sq_limit`.
> Everything else in this brief stands. See `PCB_V2/CLAUDE.md` → Gotchas.

Module: Meskernel LDJ100-689 (phase-shift / iToF, 610–690nm visible red, UART).

## Problem

Occasional wildly wrong distance readings. Not a constant offset, not
distance-proportional — most shots are fine. Correlates strongly with target
surface: matte limestone is reliable, black and/or specular surfaces produce
the bad readings.

Goal: the device should **refuse to return a number it can't trust** rather
than returning a plausible-but-wrong one. In cave survey a silently wrong shot
propagates into loop closure and isn't caught for hours. A visible "no reading"
is the desired behaviour — the user reshoots from a better angle or has an
assistant hold a white target card.

## Root cause (working theory)

Black surfaces absorb the beam; specular surfaces reflect it away from the
receiver or return light that bounced off some other surface at a different
range. Either way the return signal is weak, and the phase estimate degrades or
locks onto the wrong return. This is largely physics, not a defect — module
spec only guarantees ±3mm over reflectivity 0.2–1.0, and black paper is
0.01–0.02. The fix is detection and rejection, not elimination.

Contributing factor: cover glass over the aperture. ~4% reflection per surface
returns straight into the receiver at zero range, forming a crosstalk noise
floor. Irrelevant against bright targets, dominant against dark ones. Being
addressed separately in hardware (septum between TX/RX, 5–10° window tilt,
AR coating specced for ~650nm). Not this session's work.

## What the module already provides

### 1. Signal Quality (SQ) — the primary signal

16-bit value in **bytes 10:11 of every measurement result frame** (register
0x0022). Comes free with the distance, no extra round trip.

**SMALLER = BETTER.** Per the manual: the smaller the SQ value, the stronger
the laser signal and the more reliable the result. This is inverted from what
the name implies — easy to threshold backwards.

Manual's worked examples: 0x002C, 0x0031, 0x0033, 0x0038, 0x003C (44–60) on
50–51mm shots. Range and scale shape at longer distances / worse targets are
undocumented — must be determined empirically (see Calibration below).

### 2. Status codes — register 0x0000, separate read

Relevant ones:

| Code | Meaning |
|------|---------|
| 0x0000 | No errors |
| 0x0003 | Target out of range |
| 0x0004 | Invalid measurement value |
| 0x0005 | Excessive ambient light |
| 0x0008 | Weak laser signal |
| 0x000A | Laser signal instability |

**0x0008 is a binary flag, not a rating.** It is also *not* a guarantee that no
reading was returned — the module can hand back a distance frame that looks
completely normal while this flag sits set in a register you never read. That is
the current failure mode.

0x000A points at power supply rather than optics — see Power below.

> Note: the status-code table in the PDF text extraction has mangled row
> ordering. **Verify code→description mapping against the original manual page
> before hard-coding.**

### 3. Frame integrity

- Measurement/normal frames start **0xAA**.
- **Error/status feedback frames start 0xEE.** If the parser scans for 0xAA and
  then reads a fixed byte count, an 0xEE frame desynchronises it and subsequent
  bytes get misparsed as a distance. **Strong candidate for the current
  intermittent garbage — check this first.**
- Checksum = `(byte[1] + byte[2] + ... + byte[N]) & 0xFF`.
- Result frame layout: `[0]=0xAA, [1]=RW/addr, [2:3]=reg(0x0022),
  [4:5]=count(0x0003), [6:9]=distance (mm, big-endian 32-bit),
  [10:11]=SQ, [12]=checksum`.

## Work items, in priority order

1. **Fix frame parsing.** Validate start marker, length, and checksum on every
   frame. Handle 0xEE frames explicitly as status, not as measurements. On any
   failure, discard and re-sync by scanning forward to the next start marker —
   never read at fixed offsets after a bad frame. This may be the whole bug.

2. **Parse and surface SQ.** Extract bytes 10:11 on every measurement. Log it
   even before it's used for rejection.

3. **Switch to low-speed single measurement mode.**
   `AA 00 00 20 00 01 00 02 23`
   Automatic mode picks speed from signal strength; low-speed explicitly
   prioritises accuracy. Correct choice for survey shots on marginal targets.

4. **Implement rejection logic:**

   ```
   take 5 shots (low-speed single)
   discard any frame failing checksum/format validation
   discard any shot with SQ above threshold
   if fewer than 3 survive        -> FAIL
   d = median(survivors)
   if (max - min) > spread_limit  -> FAIL
   if d outside [0.03m, rated max] -> FAIL
   return d
   ```

   Suggested starting spread_limit: 20–30mm. Median, not mean — one outlier
   drags a mean but barely moves a median.

5. **Fail loudly.** Return a distinct error state to TopoDroid, not a number and
   not a silent retry. Distinguish "no reading — weak signal" from "no reading —
   out of range" in the user-facing message if the protocol allows; the remedies
   differ (target card vs. shorter shot).

## Calibration session — do this before fixing the threshold

Log **distance, SQ, and status code together** for every shot.

- Good matte limestone (or white card) at 2, 10, 30, 60m
- Black and/or specular surface at the same distances
- ~30 shots per condition

Determine:

- **Usable SQ range.** Manual only shows 44–60 at 50mm. Unknown what a bad
  target gives — could be 200, could be 0xFFFF.
- **Does SQ scale with distance independently of reflectivity?** Almost
  certainly yes (1/r²). A white wall at 80m will show worse SQ than the same
  wall at 5m. **A single fixed threshold would wrongly reject legitimate long
  shots** — likely need threshold as a function of distance.
- **At what SQ does 0x0008 start firing?** If SQ tracks the flag cleanly, drop
  the status poll from the hot path and reject on SQ alone (saves a round trip
  per shot — matters for battery and latency on a handheld).
- **Does the status register latch or clear on read?** Read 0x0000 after a bad
  shot, then read again without measuring. If it returns 0x0000 the second time
  it clears on read and must be polled after every measurement. If it persists,
  track/clear it or old errors get misattributed to new shots.

Plot both curves. If well separated at every range, a distance-dependent
threshold between them is the rejection rule.

## Also check

**Power.** Pin spec asks for >300mA at 3.3V headroom; continuous measurement
draws <160mA. Scope the 3.3V rail during a measurement burst — the laser firing
causes current spikes, and rail sag produces status 0x000A (laser signal
instability) and general misbehaviour. Add bulk capacitance close to the module
if dips are visible. Confirm UART logic levels are 3.3V, not 5V.

**Offset register 0x0012.** Signed 16-bit, millimetres, persists across power
cycles. `0x007B` = +123mm, `0xFF85` = -123mm. This is where the constant
path-length offset from the cover glass gets calibrated out — in the module,
not in firmware. Recalibrate with the final housing assembled, across the full
working range, not just at short distance.

## Out of scope this session

Optical hardware changes (septum, window tilt, AR coating). Those lower the
noise floor and extend how dark a target can be shot, but they don't remove the
need for rejection logic.
