# Changelog

## 0.1.0 (unreleased)

- Zone plant model (two masses, split-range heat/cool, dead time, sensor lag, noise) with four presets.
- Identification: FOPDT (two-point, least squares), lead-lag with dead time, relay analysis.
- Tuning rules (ZN, SIMC, lambda, AMIGO, Tyreus-Luyben, SIMC with Ms target) and the crossover cooling-gain ratio.
- `tcu-tune` CLI: demo, fit, batch.
- SCL blocks FB_ZonePID, FB_ZonePlant, FB_Scenario, FB_ZoneTest; OB30/DB source for TIA.
- SCL-subset translator and equivalence tests (FB_ZonePID bit-exact in float32).
- Study script, results tables and plots.
- Open: TIA Portal compile (V02) and PLCSIM trace (V03).
