"""Physical derivation of the TCU zone parameters.

The lumped model in plant.py has nine numbers per zone (Cw, Cm, UAwm, UAloss,
heater power, cooling law, dead time, sensor lag, process heat).  This module
derives them from the hardware a plant engineer can look up or measure:

* the TCU: heater power, pump flow, water volume, the plate heat exchanger of
  the closed primary circuit and its cooling-water valve;
* the hoses: length and bore (transport delay);
* the machine part: steel mass, cooling-channel geometry, outer surface;
* the process: the share of the drive power that the zone has to take away.

Heat-transfer relations (references in docs/sources/REFERENCES.md):

* water properties: tabulated values [H1, Table A.6], linear interpolation;
* channel film coefficient: Gnielinski correlation for turbulent flow
  (3000 < Re < 5e6) [H1, eq. 8.62], Petukhov friction factor [H1, eq. 8.21];
* plate heat exchanger: effectiveness-NTU, counterflow [H1, eq. 11.29a; H2];
  the cooling-water film coefficient scales with flow^0.8 [H1, eq. 8.60] and
  the overall UA is the series sum of the two films (wall neglected);
* cooling-water valve: linear (solenoid valve, pulse-width modulated) or
  equal-percentage with rangeability R [V1].

Every number used in PRESET_DESIGNS is tagged in DESIGN_SOURCES with its
status: "std" (textbook relation or property), "derived" (computed here) or
"assumed" (a sizing choice for a typical rubber-line TCU, to be replaced by
the nameplate of the real unit, see DOSSIER section 8.2).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# --------------------------------------------------------------------------
# water, saturated liquid [H1, Table A.6]; T in degC
# --------------------------------------------------------------------------
_WT = np.array([20.0, 40.0, 60.0, 80.0, 100.0])
_RHO = np.array([998.2, 992.2, 983.2, 971.8, 958.4])  # kg/m3
_CP = np.array([4.182, 4.179, 4.185, 4.197, 4.216])  # kJ/(kg K)
_MU = np.array([1.002e-3, 0.653e-3, 0.467e-3, 0.355e-3, 0.282e-3])  # Pa s
_K = np.array([0.598, 0.631, 0.654, 0.670, 0.679])  # W/(m K)

CP_STEEL = 0.46  # kJ/(kg K), carbon steel [H1, Table A.1]
K_STEEL = 45.0  # W/(m K), carbon steel at 80 degC [H1, Table A.1]


def water(T: float) -> dict[str, float]:
    """rho kg/m3, cp kJ/kgK, mu Pa s, k W/mK, Pr at temperature T (degC)."""
    T = float(np.clip(T, _WT[0], _WT[-1]))
    rho, cp, mu, k = (float(np.interp(T, _WT, a)) for a in (_RHO, _CP, _MU, _K))
    return {"rho": rho, "cp": cp, "mu": mu, "k": k, "Pr": cp * 1e3 * mu / k}


def gnielinski_h(m_dot: float, Dh: float, area: float, T: float) -> tuple[float, float]:
    """Film coefficient (W/m2K) and Reynolds number in a channel.

    m_dot kg/s, Dh hydraulic diameter m, area flow cross-section m2."""
    w = water(T)
    v = m_dot / (w["rho"] * area)
    Re = w["rho"] * v * Dh / w["mu"]
    if Re < 3000:
        Nu = 3.66  # laminar, fully developed, constant wall temperature [H1, eq. 8.55]
    else:
        f = (0.790 * math.log(Re) - 1.64) ** -2
        Nu = (f / 8) * (Re - 1000) * w["Pr"] / (1 + 12.7 * math.sqrt(f / 8) * (w["Pr"] ** (2 / 3) - 1))
    return Nu * w["k"] / Dh, Re


def eps_counterflow(NTU: float, Cr: float) -> float:
    """Effectiveness of a counterflow exchanger [H1, eq. 11.29a / 11.30a]."""
    if NTU <= 0:
        return 0.0
    if abs(1.0 - Cr) < 1e-9:
        return NTU / (1.0 + NTU)
    e = math.exp(-NTU * (1.0 - Cr))
    return (1.0 - e) / (1.0 - Cr * e)


def valve_fraction(u: float, R: float) -> float:
    """Cooling-water flow fraction at valve opening u (%), 0 at u = 0, 1 at 100.

    R = 0: linear (pulse-width-modulated solenoid valve, flow averaged).
    R > 1: equal-percentage with rangeability R, shifted so that it closes
           tight at 0 % (installed characteristic, full authority) [V1]."""
    x = min(max(u / 100.0, 0.0), 1.0)
    if R <= 1.0:
        return x
    return (R ** (x - 1.0) - 1.0 / R) / (1.0 - 1.0 / R)


def hx_conductance(f: float, UA_nom: float, Cs_max: float, Cp: float, n: float = 0.8) -> float:
    """Cooling conductance G = eps * Cmin (kW/K) of the plate heat exchanger.

    Heat removed from the primary circuit is G * (Tw - Tcw).
    f: cooling-water flow fraction (0..1), UA_nom: overall UA at full cooling
    flow (kW/K), Cs_max: cooling-water capacity rate at full flow (kW/K),
    Cp: primary (pump) capacity rate (kW/K).  The two film resistances are
    equal at full flow; the cooling-water film scales with flow^n."""
    if f <= 0.0:
        return 0.0
    hA_p = 2.0 * UA_nom
    hA_s = 2.0 * UA_nom * f ** n
    UA = 1.0 / (1.0 / hA_p + 1.0 / hA_s)
    Cs = f * Cs_max
    Cmin, Cmax = min(Cs, Cp), max(Cs, Cp)
    return eps_counterflow(UA / Cmin, Cmin / Cmax) * Cmin


# --------------------------------------------------------------------------
# hardware description -> lumped parameters
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class ZoneDesign:
    """What a plant engineer can read off the nameplates and drawings."""
    name: str
    part: str  # what the zone heats/cools
    # TCU
    heater_kW: float
    pump_lpm: float
    tcu_water_l: float  # tank + heater + pump + exchanger, primary side
    tcu_steel_kg: float  # wetted metal of the unit
    hx_UA_nom: float  # kW/K, plate exchanger at full cooling-water flow
    cw_max_lpm: float  # cooling-water flow at 100 % valve
    valve_R: float  # 0 = linear (PWM solenoid), > 1 = equal-percentage
    Tcw: float  # degC, cooling-water supply at the test
    # hoses (TCU <-> part)
    hose_len_m: float  # one way
    hose_bore_mm: float
    # machine part
    steel_kg: float
    chan_w_mm: float  # cooling channel, rectangular
    chan_h_mm: float
    chan_len_m: float
    chan_parallel: int
    cond_path_mm: float  # effective conduction length, channel wall -> lumped steel mass
    cond_area_m2: float  # conduction area
    surface_m2: float  # outer surface to ambient
    h_out: float  # W/m2K, natural convection + radiation at ~80 degC
    UA_axial: float  # kW/K, conduction to neighbouring zones / frame
    sensor: str  # "water" (TCU return line) or "metal" (sensor in the part)
    tau_s: float  # s, sensor and thermowell lag
    q_run: float  # kW, process heat into the part while the machine runs
    T_design: float = 80.0  # degC, water temperature for the property values


@dataclass
class Derivation:
    """The lumped parameters plus how each was obtained (for the dossier)."""
    params: dict[str, float]
    notes: dict[str, str] = field(default_factory=dict)


def derive(d: ZoneDesign) -> Derivation:
    w = water(d.T_design)
    m_dot = d.pump_lpm / 60e3 * w["rho"]  # kg/s
    Cp = m_dot * w["cp"]  # kW/K, primary capacity rate
    wc = water(15.0)
    Cs_max = d.cw_max_lpm / 60e3 * wc["rho"] * wc["cp"]  # kW/K

    chan_vol_l = d.chan_w_mm * d.chan_h_mm * 1e-6 * d.chan_len_m * d.chan_parallel * 1e3
    hose_vol_l = 2 * d.hose_len_m * math.pi * (d.hose_bore_mm / 2e3) ** 2 * 1e3
    Cw = (d.tcu_water_l + hose_vol_l + chan_vol_l) * 1e-3 * w["rho"] * w["cp"] + d.tcu_steel_kg * CP_STEEL
    Cm = d.steel_kg * CP_STEEL

    a, b = d.chan_w_mm * 1e-3, d.chan_h_mm * 1e-3
    area = a * b
    Dh = 4 * area / (2 * (a + b))
    h, Re = gnielinski_h(m_dot / d.chan_parallel, Dh, area, d.T_design)
    A_wet = 2 * (a + b) * d.chan_len_m * d.chan_parallel
    hA = h * A_wet / 1e3  # kW/K
    G_cond = K_STEEL * d.cond_area_m2 / (d.cond_path_mm * 1e-3) / 1e3  # kW/K
    UAwm = 1.0 / (1.0 / hA + 1.0 / G_cond)

    UAloss = d.h_out * d.surface_m2 / 1e3 + d.UA_axial

    # transport delay from the TCU (heater, exchanger) round the external loop:
    # supply hose -> channel -> return hose (the lumped Cw holds the mixing in
    # the TCU tank; the hoses and channels act as plug flow)
    q = d.pump_lpm / 60.0  # l/s
    L = (hose_vol_l + chan_vol_l) / q

    p = {
        "Cw": Cw, "Cm": Cm, "UAwm": UAwm, "UAloss": UAloss, "Pheat_max": d.heater_kW,
        "hx_UA": d.hx_UA_nom, "Cs_max": Cs_max, "Cp": Cp, "valve_R": d.valve_R,
        "Tcw": d.Tcw, "L": L, "tau_s": d.tau_s, "q_run": d.q_run,
    }
    notes = {
        "Cw": f"water {d.tcu_water_l:.0f} l TCU + {hose_vol_l:.1f} l hoses + {chan_vol_l:.1f} l channels, "
              f"+ {d.tcu_steel_kg:.0f} kg wetted steel",
        "Cm": f"{d.steel_kg:.0f} kg steel x {CP_STEEL} kJ/kgK",
        "UAwm": f"channel Re = {Re:.0f}, h = {h:.0f} W/m2K, A = {A_wet:.2f} m2 -> hA = {hA:.2f} kW/K; "
                f"steel conduction {G_cond:.2f} kW/K in series",
        "UAloss": f"{d.h_out} W/m2K x {d.surface_m2} m2 + axial {d.UA_axial} kW/K",
        "L": f"{hose_vol_l + chan_vol_l:.2f} l external loop at {d.pump_lpm:.0f} l/min",
        "Cp": f"pump {d.pump_lpm:.0f} l/min -> {Cp:.2f} kW/K",
        "Cs_max": f"cooling water {d.cw_max_lpm:.0f} l/min -> {Cs_max:.2f} kW/K",
    }
    return Derivation(p, notes)


# Four zones of a rubber-compound line served by similar TCUs.  The hardware
# values are sizing assumptions for this class of machine (see DESIGN_SOURCES);
# the relations that turn them into model parameters are textbook.
PRESET_DESIGNS: dict[str, ZoneDesign] = {
    "A": ZoneDesign(
        "A", "cold-feed extruder, barrel zone 2 (sensor in the TCU return line)",
        heater_kW=12.0, pump_lpm=40.0, tcu_water_l=12.0, tcu_steel_kg=15.0,
        hx_UA_nom=0.9, cw_max_lpm=20.0, valve_R=0.0, Tcw=15.0,
        hose_len_m=4.0, hose_bore_mm=25.0,
        steel_kg=320.0, chan_w_mm=20.0, chan_h_mm=10.0, chan_len_m=6.0, chan_parallel=1,
        cond_path_mm=20.0, cond_area_m2=0.60, surface_m2=0.9, h_out=11.0, UA_axial=0.03,
        sensor="water", tau_s=4.0, q_run=9.0),
    "B": ZoneDesign(
        "B", "cold-feed extruder, screw (rotary union, sensor in the TCU return line)",
        heater_kW=9.0, pump_lpm=30.0, tcu_water_l=9.0, tcu_steel_kg=12.0,
        hx_UA_nom=0.6, cw_max_lpm=15.0, valve_R=30.0, Tcw=15.0,
        hose_len_m=3.0, hose_bore_mm=20.0,
        steel_kg=190.0, chan_w_mm=14.0, chan_h_mm=14.0, chan_len_m=5.0, chan_parallel=1,
        cond_path_mm=25.0, cond_area_m2=0.35, surface_m2=0.0, h_out=0.0, UA_axial=0.025,
        sensor="water", tau_s=3.0, q_run=7.0),
    "C": ZoneDesign(
        "C", "extruder head and die (sensor in the steel)",
        heater_kW=18.0, pump_lpm=50.0, tcu_water_l=16.0, tcu_steel_kg=20.0,
        hx_UA_nom=1.4, cw_max_lpm=30.0, valve_R=0.0, Tcw=15.0,
        hose_len_m=6.0, hose_bore_mm=25.0,
        steel_kg=560.0, chan_w_mm=16.0, chan_h_mm=12.0, chan_len_m=8.0, chan_parallel=2,
        cond_path_mm=40.0, cond_area_m2=1.20, surface_m2=1.6, h_out=11.0, UA_axial=0.05,
        sensor="metal", tau_s=5.0, q_run=14.0),
    "D": ZoneDesign(
        "D", "internal mixer, chamber side walls (sensor in the TCU return line, batch load)",
        heater_kW=24.0, pump_lpm=80.0, tcu_water_l=20.0, tcu_steel_kg=25.0,
        hx_UA_nom=2.6, cw_max_lpm=60.0, valve_R=30.0, Tcw=15.0,
        hose_len_m=8.0, hose_bore_mm=32.0,
        steel_kg=900.0, chan_w_mm=25.0, chan_h_mm=12.0, chan_len_m=10.0, chan_parallel=2,
        cond_path_mm=30.0, cond_area_m2=1.80, surface_m2=2.5, h_out=11.0, UA_axial=0.06,
        sensor="water", tau_s=6.0, q_run=17.0),
}

DESIGN_SOURCES: dict[str, str] = {
    "water properties, cp/k of steel": "std [H1 Tables A.1, A.6]",
    "Gnielinski, Petukhov, Dittus-Boelter exponent 0.8": "std [H1 eq. 8.21, 8.60, 8.62]",
    "counterflow effectiveness-NTU": "std [H1 eq. 11.29a], [H2 ch. 3]",
    "equal-percentage valve characteristic": "std [V1]",
    "heater 9-24 kW, pump 30-80 l/min, tank 9-20 l": "assumed (water TCU class for extruders and mixers)",
    "plate exchanger UA 0.6-2.6 kW/K at full cooling flow": "assumed (sized to remove q_run at 25-60 % valve)",
    "cooling-water valve type per zone": "assumed (A, C: PWM solenoid; B, D: proportional, R = 30)",
    "hose length/bore": "assumed",
    "steel mass, channel geometry, surfaces": "assumed (barrel zone of a 150 mm class extruder, mixer side wall)",
    "outer film 11 W/m2K": "std [H1 ch. 9 natural convection ~5 + radiation ~6 at 80 degC, eps 0.8]",
    "sensor lag 3-6 s": "assumed (Pt100 in a thermowell in flowing water)",
    "process heat q_run 7-17 kW": "derived/assumed: about 40 % of the compound heating power of a "
                                  "150 mm class extruder (project 12: 89 kW at 30 rpm) shared over the zones; "
                                  "mixer wall share of the batch energy assumed",
    "cooling water 8-30 degC over the year": "assumed (cooling-tower circuit; chiller circuits narrower)",
}
