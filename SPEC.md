# SPEC: TCU zone identification, PI tuning and SCL controller

Rev A, 9 Nov 2026. Status of each requirement: see [Traceability](#traceability).

## 1. Purpose

A water temperature control unit (TCU) feeds several zones of a machine (an
extruder barrel, a screw, a mixer roll...) with heated or cooled water. Each zone
has a heater and a cooling valve and one PID loop in a Siemens S7 PLC. Typical
complaints are oscillation in some operating states and settings copied from
zone to zone without testing.

This project builds, on simulated zones:

1. a plant model of one zone that is realistic enough to show those problems,
2. identification from the tests a technician can record on a real machine,
3. a tuning tool that turns those records into controller settings,
4. the controller itself as a Siemens SCL function block, and
5. evidence that the SCL source does what the Python reference does.

Everything is simulated and generic. No data from any real machine is used.

## 2. Requirements

| ID | Requirement |
|---|---|
| F01 | Plant model of one zone: two thermal masses (water, machine part), electric heater, cooling valve with heat removal proportional to (Tw − Tcw), transport dead time, sensor lag, noise and resolution, process heat input. Four preset zones of different size; sensor on the water or in the machine part. |
| F02 | Test records as CSV (`t_s, heat_pct, cool_pct, pv_degC`): heater step, cooling-valve step under process load, relay test. |
| F03 | Identification from a step record: FOPDT (two-point and least squares) and a five-parameter lead-lag model with dead time; RMSE of each fit reported. |
| F04 | Relay test analysis (Åström–Hägglund) with a hysteresis correction: Ku, Pu. |
| F05 | Tuning rules in the FB's parameter form: Ziegler–Nichols (step), SIMC, lambda, AMIGO, Tyreus–Luyben (relay), and SIMC on the lead-lag model with τc raised until Ms ≤ target. A rule whose assumptions fail (e.g. no visible dead time) says "not applicable" instead of giving numbers. |
| F06 | Cooling-gain ratio for the split-range output: static (Kcool/Kheat) and at the loop's gain crossover. |
| F07 | Robustness: maximum sensitivity Ms of every candidate in the heating and in the cooling range. |
| F08 | `FB_ZonePID` (SCL): PI(D) in ISA form, setpoint weight, derivative on the measurement with filter, back-calculation anti-windup, bumpless manual/auto, reset, split range heat/cool with CoolGainRatio, deadband, sample time as an input. |
| F09 | `FB_ZonePlant` (SCL): the F01 model (no noise) so the loop can run in S7-PLCSIM without hardware. |
| F10 | Command-line tool `tcu-tune` with `demo`, `fit` and `batch` (all zones of a folder, one summary CSV). |
| F11 | Production scenario (cold start, setpoint up, process load on, setpoint down while cooling, load off) with metrics: IAE, overshoot, load peak, settling, actuator travel, heat/cool switchovers. Same profile in Python and in `FB_Scenario`. |
| V01 | The SCL source text is executed in CI (by `tcutune.scl`) and matches the Python reference: `FB_ZonePID` bit-exact in float32, `FB_ZonePlant` within 0.01 K, the whole SCL test program within 0.05 K. |
| V02 | The SCL sources compile in TIA Portal with 0 errors. |
| V03 | A PLCSIM trace of `FB_ZoneTest` matches the Python closed loop. |

## 3. Acceptance targets

| Target | Check |
|---|---|
| FOPDT fit on a pure FOPDT record recovers K ±2 %, T ±5 %, L ±3 s | `test_fopdt_fit_recovers_true_model` |
| Lead-lag fit on a TCU zone has < 60 % of the FOPDT residual and shows the dead time again | `test_lead_lag_fit_beats_fopdt_on_tcu_zone` |
| Noise-free relay test: Ku ±10 %, Pu ±5 % of the true ultimate point | `test_relay_finds_ultimate_point_without_noise` |
| Recommended setting meets Ms ≤ 1.4 on the identified model | `test_simc_ms_meets_target_on_model`, `test_cli.py` |
| On the true plant, Ms in the cooling range with the tool's ratio is within 0.05 of Ms in heating | `results/study.md` (all four zones) |

## 4. Out of scope

* PWM (time-proportioning) heater outputs and valve hysteresis; the outputs are continuous percent.
* Gain scheduling over temperature (the cooling gain grows with Tw − Tcw; one ratio is used).
* Data from real machines; the tool reads the same CSV a real test would produce, but it has only seen simulated records.
* Siemens PID_Temp. It also has a heating/cooling factor; this project uses its own FB so that the same code can be read, tested and compared line by line.

## Traceability

| ID | Implementation | Verification | Status |
|---|---|---|---|
| F01 | `src/tcutune/plant.py` | `test_steady_state_matches_long_simulation`, `test_euler_step_is_accurate`, `test_linear_frf_dc_gain` | done |
| F02 | `experiments.py` (`heat_step_test`, `cool_step_test`, `relay_test`, `StepRecord`) | `test_cli.py` | done |
| F03 | `identify.py` (`two_point`, `fit_fopdt`, `fit_lead_lag`) | `test_fopdt_fit_recovers_true_model`, `test_lead_lag_fit_beats_fopdt_on_tcu_zone` | done |
| F04 | `identify.relay_analysis` | `test_relay_finds_ultimate_point_without_noise`, `test_relay_with_noise_and_hysteresis_is_conservative` | done, limit documented |
| F05 | `tuning.py` | `test_textbook_values`, `test_rules_reject_missing_dead_time`, `test_simc_ms_meets_target_on_model` | done |
| F06 | `tuning.crossover_ratio`, `tool.tune_zone` | `test_crossover_ratio_equals_static_for_same_dynamics`, study | done |
| F07 | `tuning.max_sensitivity` | study (true-plant Ms) | done |
| F08 | `plc/FB_ZonePID.scl`, reference `pid.py` | `test_pid.py`, `test_pid_bit_exact_random_inputs` | done |
| F09 | `plc/FB_ZonePlant.scl` | `test_plant_matches_python_model` | done |
| F10 | `cli.py`, `tool.py` | `test_demo_fit_batch` | done |
| F11 | `experiments.closed_loop`, `metrics.py`, `plc/FB_Scenario.scl` | `test_scenario_profile`, study | done |
| V01 | `scl.py` | `test_scl_equivalence.py` | done |
| V02 | `plc/*.scl`, `plc/tia/ZoneTest_OB_DB.scl` | TIA Portal compile | **open** |
| V03 | `FB_ZoneTest` + OB30 | PLCSIM trace vs Python | **open** |
