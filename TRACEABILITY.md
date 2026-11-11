# Traceability: requirement → implementation → verification → evidence

The requirements are defined in [DOSSIER.md §4](DOSSIER.md). Verification methods are I (inspection),
A (analysis), D (demonstration) and T (test), following [W] p.270–272. This is the requirements
verification traceability matrix of DOSSIER §11.1, extended with the code and test locations.
Status on 11 Nov 2026; everything is simulation.

| Req. | Need | Implemented in | Verified by | Evidence | Status |
|---|---|---|---|---|---|
| SPS-101 ±1 K ≥ 99 % of time | N1 | whole loop | T: line trial VAL-1 | — | open (plant); the study runs scenarios, not production weeks |
| SPS-102 load-step peak ≤ 2 K | N1 | `FB_ZonePID`, tuning, `FB_CoolLinearise` | A: study §3 (test point and corners) | results/study.md §3, `time_domain.csv` | met by E on A, B; D 1.8–2.2 K; head zone C not met (DOSSIER §13) |
| SPS-103 SP-down overshoot ≤ 1 K | N1 | ratio / linearisation, conditional integration | A: study §3 | results/study.md §3 | met by C everywhere and by E on zones A, B, D (head zone C with E: 1.2 K); not by A, B |
| SPS-104 no heater/valve limit cycle | N3 | ratio / linearisation | A: study §3 | results/study.md §3 | C, E met; A, B fail (limit cycles) |
| SPS-105 batch ripple ≤ ±0.5 K | N1 | `FB_ZonePID.FeedForward` | A: study §6 | results/study.md §6, `docs/img/batch_feedforward.png` | met with and without FF; FF halves it |
| SPS-201 Ms ≤ 1.6 over the envelope | N2 | `FB_CoolLinearise` mode 2 + crossover ratio | A: `envelope.evaluate` | results/study.md §5, `docs/img/envelope_ms.png` | E met on all zones, clean and fouled (1.19–1.41); C up to 1.70 |
| SPS-202 tool from 3 tests | N4 | `tcu-tune` | D: `tests/test_cli.py` | pytest | met |
| SPS-203 refuse invalid rules | N4 | `tuning._check_fopdt` | T: `test_rules_reject_missing_dead_time` | pytest | met |
| SPS-204 bumpless manual | N3 | `FB_ZonePID` | T: `test_pid.py`, bit-exact SCL test | pytest | met |
| SPS-205 conditional integration | SPS-103 | `FB_ZonePID.CoolSaturated` | T: `test_conditional_integration_when_cooling_valve_saturates`, SCL bit-exact test | pytest | met |
| SPS-206 Tcw sensor fault holds | FMEA F3 | `FB_CoolLinearise.TcwValid` | T: `test_cool_linearise_holds_tcw_on_sensor_fault` | pytest | met in the translator; PLCSIM open |
| SPS-301 one new signal per TCU | N5 | `FB_CoolLinearise.Tcw` | I | DOSSIER §6.2 | met |
| SPS-302 FF from existing power signal | N5 | `FB_ZonePID.FeedForward` | I | DOSSIER §6.2 | met |
| SPS-361 TIA V18 0/0 | constraint | `plc/`, `plc/tia/` | T: TIA Openness compile | results/tia_compile*.txt, docs/img/tia_FB_CoolLinearise.png | met |
| SPS-362 ≤ 20 % of OB30 with 3× margin | constraint | all FBs | A: static count (study §7); T: CPU statistics (VT-7) | results/study.md §7 | A met (estimate); T open |
| SPS-363 SCL = Python reference | constraint | `src/tcutune/scl.py` | T: `tests/test_scl_equivalence.py` | pytest in CI | met |

## Code-to-requirement index

| File | Requirements |
|---|---|
| `plc/FB_ZonePID.scl` | SPS-101…105, 204, 205 |
| `plc/FB_CoolLinearise.scl` | SPS-201, 205, 206, 301 |
| `plc/FB_ZonePlant.scl`, `FB_Scenario.scl`, `FB_ZoneTest.scl`, `plc/tia/*` | SPS-361…363 (test harness) |
| `src/tcutune/physics.py`, `plant.py` | model behind SPS-201 (DOSSIER §8) |
| `src/tcutune/envelope.py` | SPS-201 analysis |
| `src/tcutune/tool.py`, `cli.py`, `identify.py`, `tuning.py` | SPS-202, 203 |
