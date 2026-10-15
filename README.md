# JalMaa — Water-Birth Monitor Digital Twin

Continuous mother-and-baby monitoring during water birth (L&T TECHgium "Aqua Birth" problem).

Two sealed body pods on the mother talk by **magnetic induction (MI)** to **relays at the pool waterline**. The relays forward the data over BLE in air to a bedside tablet that shows a CTG-style view with screening alarms. This repository contains a working simulation of the whole chain, used to size the design and to stage the jury demo.

## Architecture

```
Wrist pod M1 ─┐                                      
(PPG HR/SpO2, ├─ MI 3.39 MHz, TDMA 250 ms, CRC+ARQ ─▶ Relays R1 + R2 ─ BLE ─▶ Bedside tablet
 PTT, IMU)    │   (under water, ≤ 1.7 m)              (opposite walls,        (CTG view, decel
Abdominal M2 ─┘                                        coil 15 cm deep,        classifier,
(Doppler FHR, abdominal ECG/EHG,                       water temp)             3-tier alarms)
 strain toco, insulated ZHF temp)
```

## What is in here

| Path | Contents |
|---|---|
| `web/index.html` | The digital twin. Open it in any browser — no install, no server. |
| `sim/engine.js` | Simulation engine: MI link physics, 2.4 GHz water attenuation, synthetic maternal/fetal physiology, pool kinematics, relay TDMA network, bedside detection and alarms, validation tests. Runs in the browser and in Node. |
| `tests/run-validation.js` | Runs the 15-test validation suite from the command line. |
| `web/ui-template.html`, `build.py` | UI source; `python3 build.py` inlines the engine to regenerate `web/index.html`. |

## Run

```bash
# Interactive twin
open web/index.html            # or double-click it

# Validation suite (Node 16+)
node tests/run-validation.js 7     # seed is optional
```

## Validation results (seed 7; all tests also pass on seeds 100–129)

| Test | Target | Result |
|---|---|---|
| MI coverage, 2 relays, tri-axis coils | ≥ 99 % positions, PER < 1 % | 100 % |
| Coverage in contaminated water (0.5 S/m) | ≥ 99 % | 100 % |
| Original plan (1 relay, single coil, 13.56 MHz) in same water | reference | ≈ 66 % |
| BLE 2.4 GHz failure depth | < 30 cm | ≈ 20 cm |
| End-to-end latency | p95 < 500 ms | ≈ 136 ms |
| Data completeness | ≥ 99.5 % | 100 % |
| Contraction detection with artefacts | Sens/PPV ≥ 95 %, onset ≤ 10 s | 100 % / 100 %, ≈ 4 s |
| Recurrent late decelerations | urgent flag ≤ 15 min | ≈ 7 min |
| Fetal tachycardia / reduced variability / tachysystole | flag ≤ 15 min | 8–12 min |
| Maternal pulse recorded as FHR | flag ≤ 60 s | ≈ 25 s |
| Maternal desaturation | urgent when SpO₂ < 90 % | flagged |
| Pod removed → alarm → reconnect | ≤ 6 s / ≤ 2 s | 6 s / 0.25 s |
| False alarms, 60 min normal labour | 0 | 0 |
| Battery endurance | ≥ 12 h per pod | 47 h / 31 h |

## Key design decisions

- **3.39 MHz carrier** (inductive short-range band — confirm with WPC; 6.78 MHz ISM as fallback): keeps the whole pool inside the near field and holds margin when water conductivity rises. 13.56 MHz loses ~21 dB to conduction at 1.5 m and leaves the near field at 0.41 m.
- **Two relays + three-axis pod coils** remove the far-side distance and orientation nulls.
- **250 ms TDMA superframe** with CRC and retransmission instead of 1 s bursts.
- **Low-power MCU in the pods** (nRF52 / STM32L4 class); ESP32 only in the relays.
- **Sealed adhesive skirts for abdominal electrodes**, maternal ECG for PTT timing, relay sync beacon for pod clocks.
- **Water temperature at the relay + insulated ZHF module** for maternal temperature (a skin thermistor in a 37 °C pool reads the water).
- **FHR vs maternal HR coincidence check** against Doppler mis-registration.
- Contractions declared only when **EHG and strain agree** (motion-artefact rejection).

## Limits

The physiology is synthetic and the link model is first order. Next step: measure the real coil pair in a tub of warm tap water (and 0.5 S/m saline) and feed the measured Q, noise floor and path loss into the Link budget tab.

Not a medical device. All clinical outputs are screening flags that a midwife must confirm.
