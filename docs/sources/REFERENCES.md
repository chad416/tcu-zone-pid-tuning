# References and source status

Every value or relation in this repo has one of these source states:
- **[read]**: the source was opened and the cited value comes from it.
- **[std]**: a standard textbook result or property. The section is named; the page was not re-checked for this repo.
- **[derived]**: computed in this repo from [std] relations and the inputs listed with it.
- **[assumed]**: a sizing or modelling choice of this repo. All of them are listed in DOSSIER section 8.2 with the sensitivity that was run.

## Systems engineering

- **[W]** C. S. Wasson, *System Engineering Analysis, Design, and Development: Concepts, Principles,
  and Practices*, 2nd ed., Wiley, 2016. **[read]**, ch. 4–7, 13, 19–22, 26–28, 31–34. Reading notes with
  printed and PDF page numbers are in `wasson_extract_ch04-22.md` and `wasson_extract_ch26-34.md`.

## Heat transfer and the temperature control unit

- **[H1]** T. L. Bergman, A. S. Lavine, F. P. Incropera, D. P. DeWitt, *Fundamentals of Heat and Mass
  Transfer*, 7th ed., Wiley, 2011. **[std]**
  - Table A.1 (carbon steel cp 0.46 kJ/kgK, k about 45 W/mK at 80 °C), Table A.6 (saturated water).
  - Eq. 8.21 (Petukhov friction factor), 8.60 (Dittus–Boelter, Re^0.8), 8.62 (Gnielinski), 8.55
    (laminar Nu = 3.66).
  - Ch. 9: natural convection; with radiation at about 80 °C and emissivity 0.8 the outer film is about
    11 W/m²K (5 + 6).
  - Eq. 11.29a/11.30a: counterflow effectiveness–NTU.
- **[H2]** R. K. Shah, D. P. Sekulić, *Fundamentals of Heat Exchanger Design*, Wiley, 2003, ch. 3 and 7
  (ε-NTU; plate exchangers; film coefficients scale with Re^0.6–0.8). **[std]**
- **[H3]** TEMA, *Standards of the Tubular Exchanger Manufacturers Association*, 9th ed., 2007, Table
  RGP-T-2.4: fouling resistance of treated cooling-tower water about 0.000176 m²K/W (0.001 h ft² °F/Btu).
  **[std]** (used only as the order of magnitude for an end-of-cleaning-interval case).
- **[V1]** ISA-75.11.01, *Inherent Flow Characteristic and Rangeability of Control Valves*; equal-percentage
  characteristic f = R^(x−1). **[std]**

## Rubber processing (where the process heat comes from)

- **[R1]** Project 12 of this portfolio, `extruder-pressure-fill-estimator`: 150 mm class cold-feed
  extruder model, 741 kg/h at 30 rpm, 0.12 kWh/kg, i.e. about 89 kW into the compound. **[derived]**
  (its own sources: Rauwendaal, Tadmor & Gogos, Ha et al. 2007).
- **[R2]** J. L. White, *Rubber Processing: Technology, Materials, and Principles*, Hanser, 1995: internal
  mixers run in batch cycles of a few minutes with cooled chamber walls and rotors; extruder barrel and
  screw zones are temperature-conditioned with water. **[std]** No numbers taken.

## Control

- **[C1]** K. J. Åström, T. Hägglund, *Advanced PID Control*, ISA, 2006. **[std]** Ms as the robustness
  measure (1.2–2.0 usual, 1.4 robust), back-calculation and conditional integration anti-windup, relay
  auto-tuning, AMIGO.
- **[C2]** S. Skogestad, "Simple analytic rules for model reduction and PID controller tuning",
  *J. Process Control* 13 (2003) 291–309. **[std]** SIMC rules and the half rule.
- **[C3]** S. Skogestad, I. Postlethwaite, *Multivariable Feedback Control*, 2nd ed., Wiley, 2005,
  sec. 5.2–5.3. **[std]** Bode sensitivity integral: |S| < 1 at low frequency forces |S| > 1 somewhere
  else (the "waterbed").
- **[C4]** F. G. Shinskey, *Process Control Systems*, 4th ed., McGraw-Hill, 1996. **[std]** Split-range
  heat/cool loops, gain compensation of nonlinear final elements, feedforward with dynamic compensation.
- **[C5]** W. L. Luyben, "Tuning proportional-integral controllers for processes with both inverse
  response and deadtime", and Tyreus–Luyben settings from a relay test. **[std]**
- **[C6]** L. Ljung, *System Identification: Theory for the User*, 2nd ed., 1999, output-error models. **[std]**

## PLC

- **[P1]** Siemens data sheet 6ES7513-1AL02-0AB0 (CPU 1513-1 PN): typical 256 ns per floating-point
  instruction, 48 ns per word instruction. **[read]** (read for project 12 from the SIPLUS twin data sheet
  6AG1513-1AL02-2AB0, 10 Apr 2023).
- **[P2]** Siemens TIA Portal V18, STEP 7 Professional; compile results through TIA Openness on this PC.
