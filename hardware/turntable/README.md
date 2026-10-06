# Motorised insect turntable (NEMA 17 + A4988, USB serial)

Parts (user, 2026-09-28): Purecrea NEMA 17 **17HS4401** (42x42x40 mm, 1.8 deg = 200 steps/rev,
1.7 A/phase, 0.45 Nm) + Purecrea **A4988** driver module. Not decided yet: the board that
runs the driver, the coupling motor -> platter (direct or belt/gears), microstepping, the
number of stops per revolution. All of those are plan keys, nothing is hard-coded.

## Why a small board between the USB port and the A4988

The A4988 wants a STEP pulse per microstep and a DIR level; a PC's USB port cannot toggle
pins. Any Arduino-class board with a USB serial port does it (Uno, Nano, Leonardo, Pro
Micro, ...; the sketch uses nothing but `digitalWrite`). Flash `turntable_a4988.ino` with
the Arduino IDE once; from then on the coordinator talks to it over USB serial
(`coordinator/capture_coordinator/turntable.py`, protocol at the top of the sketch).

## Wiring

```
A4988            ->
  VMOT, GND      -> motor supply 8-35 V (12 V typical) with a 100 uF electrolytic ACROSS VMOT/GND, close to the module
  VDD, GND       -> board 5 V, board GND (logic; also ties the grounds together)
  1A 1B 2A 2B    -> the two motor coils (17HS4401: one coil per pair of leads that show
                    ~1.5 ohm between them; swapping a pair only flips the direction)
  STEP           -> board pin 3     (PIN_STEP in the sketch)
  DIR            -> board pin 4     (PIN_DIR)
  ENABLE         -> board pin 5     (PIN_EN, active LOW; the sketch keeps it LOW = holding)
  SLEEP + RESET  -> tie together (RESET floats otherwise and the driver does nothing)
  MS1 MS2 MS3    -> microstepping: all open = full step; H H H = 1/16 (default in the plan)
```

**Current limit** (trimmer on the A4988, measure Vref between the trimmer wiper and GND with the
motor unplugged): `I_max = Vref / (8 * R_sense)`. Most modules have R_sense = 0.1 ohm (marked
R100), some 0.05 (R050). For the 17HS4401 set about 1.2 A (70 % of rated, plenty for a
platter and no overheating without a fan): Vref ≈ 0.96 V with R100, 0.48 V with R050. Above
about 1 A the A4988 needs its heatsink. The platter holds position while enabled (`turntable_hold`);
disabling the driver lets the rotor snap to the nearest full-step detent (up to 0.9 deg at
1/16), so the coordinator keeps it enabled during a session and releases it at the end.

Never plug or unplug the motor while the driver is powered (kills the A4988).

## Plan keys

```yaml
turntable_drive: serial          # manual (Enter, as before) | serial | fake (dry run)
turntable_port: auto             # or /dev/ttyACM0 (Ubuntu), /dev/cu.usbmodem14101 (Mac)
turntable_baud: 115200
turntable_steps_per_rev: 200     # motor full steps (17HS4401: 1.8 deg)
turntable_microsteps: 16         # MS1..3 jumpers
turntable_gear_ratio: 1.0        # motor revolutions per PLATTER revolution (belt 20:60 = 3.0)
turntable_direction: 1           # -1 flips the sense
turntable_speed_deg_s: 20        # platter speed
turntable_accel_deg_s2: 40
turntable_hold: true             # driver enabled between moves
turntable_steps: 36              # stops per revolution (unchanged), turntable_step_deg: 10.0
turntable_confirm: enter         # with a motor: enter = no gate (yaw logged), mocap | both = yaw gate ± tol
```

Resolution: 360 / (steps_per_rev * microsteps * gear_ratio) per microstep = 0.1125 deg at
1/16 direct drive; targets are absolute and rounded once per command, so the position error
at any stop is at most half a microstep (a 10 deg stop is 88.9 microsteps, that is fine).

## Session flow with the motor

1. Robot at the initial pose, "Place/check the insect, then Enter" (the only manual action);
   the coordinator sets the board's position to `step * step_deg` (0 at a fresh start, the
   resumed step otherwise, so `--resume --start-step N` works: put the platter at stop N first).
2. Every further rotation: `GOTO step * step_deg`, wait for the board's OK, then the usual
   `settle_after_move_s` and, if `turntable_confirm` is `mocap`/`both`, the popillia yaw gate
   (the sign of the yaw change is learnt at the first move and logged).
3. Serial error or timeout -> retry / abort prompt. End of session: driver released.

Bench test without the coordinator (inside `optitrack/`, pyserial is in the pixi env):

```bash
pixi run python -m capture_coordinator.turntable --port auto pos
pixi run python -m capture_coordinator.turntable goto 10          # absolute
pixi run python -m capture_coordinator.turntable cycle 36 10 0.5  # a whole revolution like a session
```
(run from the repo root with `PYTHONPATH=coordinator`.)
