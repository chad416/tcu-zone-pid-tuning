# Changelog

## 0.2.0 (11 Nov 2026)

- **Physics instead of invented presets.** `physics.py` derives every zone parameter from hardware a
  plant engineer can look up: heater, pump flow, water volume, hoses, channel geometry (Gnielinski
  film coefficient with steel conduction in series), steel mass, outer surface, process heat.
- **Closed primary circuit with a plate heat exchanger.** Cooling is now ε-NTU (counterflow) behind a
  linear (PWM solenoid) or equal-percentage valve; the bilinear direct-cooling law stays as an option.
  Fouling by a TEMA-order fouling resistance.
- **New block `FB_CoolLinearise`** (SCL): mode 0 off, mode 1 Tw−Tcw scaling, mode 2 inversion of the
  valve + exchanger model by bisection; holds the last valid cooling-water temperature on a sensor
  fault; reports saturation.
- **`FB_ZonePID` 0.2:** lagged feedforward input; conditional integration when the linearised valve
  saturates (found as windup in the summer corner).
- **`FB_ZonePlant` 0.2:** exchanger cooling in SCL (EXP/LN), cooling-water input. `FB_ZoneTest` runs the
  linearisation and the feedforward.
- **Envelope analysis** (`envelope.py`): Ms in cooling over setpoint × load × cooling water × fouling
  for five cooling alternatives; time-domain corners; mixer batch load and feedforward; PLC budget.
- **Tool:** `tcu-tune nameplate` prints the FB_CoolLinearise settings; `demo --lin 2` records the
  cooling test through the linearisation.
- **Dossier** (`DOSSIER.md`, `docs/DOSSIER.pdf`) on the Wasson spine; `TRACEABILITY.md`; trade study
  (`scripts/aoa.py`). SPEC.md is replaced by the dossier requirements.
- **TIA Portal V18:** all FBs, DBs and OB30 at 100 ms compile with 0 errors and 0 warnings. One warning
  on the way: `100.0 * 0.5 * (…)` is folded into an LReal constant (fixed, decision D7).
- Translator: EXP and LN.

## 0.1.0 (9 Nov 2026)

- Zone plant model (two masses, split-range heat/cool, dead time, sensor lag, noise) with four presets.
- Identification: FOPDT (two-point, least squares), lead-lag with dead time, relay analysis.
- Tuning rules (ZN, SIMC, lambda, AMIGO, Tyreus-Luyben, SIMC with Ms target) and the crossover
  cooling-gain ratio.
- `tcu-tune` CLI: demo, fit, batch.
- SCL blocks FB_ZonePID, FB_ZonePlant, FB_Scenario, FB_ZoneTest; OB30/DB source for TIA.
- SCL-subset translator and equivalence tests (FB_ZonePID bit-exact in float32).
- TIA Portal V18 build through Openness: 0 errors, 0 warnings.
