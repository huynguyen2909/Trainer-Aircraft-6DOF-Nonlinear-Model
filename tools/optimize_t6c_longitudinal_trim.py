#!/usr/bin/env python3
"""Provisional one-snapshot longitudinal trim calibration against FDR 173.txt.

Runs SciPy least_squares against the project's C++ force/moment components.
Because FDR lacks net thrust and flight mass, results require an explicit
assumed mass and elevator degree-unit assumption; they are NOT a 2.c.5 pass.
"""
import argparse
import csv
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

START, END = 13962, 14027
SEED = {"wing.CL0": 0.20287913876294228,
        "wing.Cm0": -0.062220325741653273,
        "propeller.effective_thrust_scale": 1.0}
TOLERANCES = {"elevator_angle_deg": 1.0, "pitch_angle_deg": 1.0,
              "pitch_trim_angle_deg": 0.5}


def extract_snapshot(path):
    """Read only the requested timestamps; check the export's column names."""
    keys = ["ASPD TRUE", "MACH-1", "PITCH-1", "ELEVPOS1", "ELETRIM", "NP",
            "FLAPOS-", "WOW", "IVS-1", "AOA-2"]
    rows = []
    with path.open(encoding="utf-8-sig", errors="replace") as stream:
        next(stream)
        headers = [x.strip() for x in next(stream).split(",")]
        next(stream)
        units = [x.strip() for x in next(stream).split(",")]
        indices = {k: headers.index(k) for k in keys}
        if len(headers) != len(set(headers)):
            # Duplicate names elsewhere are harmless if requested names are unique.
            for k in keys:
                if headers.count(k) != 1:
                    raise ValueError(f"Ambiguous column {k}")
        for raw in stream:
            fields = raw.rstrip("\r\n").split(",")
            try:
                hour, minute, second = (int(v) for v in fields[0].strip().split(":"))
            except (ValueError, IndexError):
                continue
            t = hour * 3600 + minute * 60 + second
            if START <= t <= END:
                rows.append((t, {k: fields[i].strip() for k, i in indices.items()}))
    if [t for t, _ in rows] != list(range(START, END + 1)):
        raise ValueError("Incomplete, duplicated or unordered FDR window")
    if {row["FLAPOS-"] for _, row in rows} != {"Up"} or \
       {row["WOW"] for _, row in rows} != {"Air"}:
        raise ValueError("Window is not clean airborne flight")
    means = {k: float(np.mean([float(row[k]) for _, row in rows])) for k in
             ["ASPD TRUE", "MACH-1", "PITCH-1", "ELEVPOS1", "ELETRIM", "NP", "IVS-1", "AOA-2"]}
    return means, {k: units[i] for k, i in indices.items()}


class Bridge:
    def __init__(self, executable, mass, speed, rho, mach, np_percent):
        self.process = subprocess.Popen([str(executable)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, text=True, bufsize=1)
        self.fixed = [mass, speed, rho, mach, np_percent]
        self.calls = 0

    def evaluate(self, params, theta_deg, elevator_deg):
        values = [*params, theta_deg, elevator_deg, *self.fixed]
        self.process.stdin.write(" ".join(f"{v:.17g}" for v in values) + "\n")
        self.process.stdin.flush()
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError("C++ trim bridge exited without a force/moment result")
        self.calls += 1
        result = np.fromstring(line, sep=" ")
        if result.size != 4 or not np.isfinite(result).all():
            raise RuntimeError(f"Bad C++ trim bridge response {line!r}")
        return result

    def close(self):
        self.process.stdin.close()
        if self.process.wait(timeout=10):
            raise RuntimeError("C++ trim bridge failed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fdr", required=True, type=Path, help="Original 173.txt export")
    parser.add_argument("--bridge", required=True, type=Path, help="Compiled tools/t6c_trim_loads.cpp")
    parser.add_argument("--mass-kg", required=True, type=float,
                        help="Explicit scenario mass; not available in FDR")
    parser.add_argument("--assume-elevator-degrees", action="store_true",
                        help="ELEVPOS1 has no unit header: explicitly assume degrees")
    parser.add_argument("--rho", type=float, default=1.10681, help="Density proxy, kg/m^3")
    parser.add_argument("--target-net-thrust-n", type=float, default=None,
                        help="Independent measured net thrust, if available")
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    if not args.assume_elevator_degrees:
        parser.error("Specify --assume-elevator-degrees after checking ELEVPOS1 units")
    if args.mass_kg <= 0 or args.rho <= 0:
        parser.error("mass and rho must be positive")
    if args.target_net_thrust_n is not None and args.target_net_thrust_n <= 0:
        parser.error("target thrust must be positive")
    obs, units = extract_snapshot(args.fdr)
    with args.fdr.open("rb") as stream:
        fdr_sha256 = hashlib.file_digest(stream, "sha256").hexdigest()
    v = obs["ASPD TRUE"] * 0.5144444444444445
    targets = {"elevator_angle_deg": obs["ELEVPOS1"],
               "pitch_angle_deg": obs["PITCH-1"],
               "net_thrust_n": args.target_net_thrust_n}
    w = args.mass_kg * 9.80665
    params0 = np.array(list(SEED.values()))
    bridge = Bridge(args.bridge.resolve(), args.mass_kg, v, args.rho,
                    obs["MACH-1"], obs["NP"])

    def balance(parameters, pitch, elevator):
        fx, fz, my, _ = bridge.evaluate(parameters, pitch, elevator)
        return np.array([fx / w, fz / w, my / (w * 1.65)])

    def trim(parameters):
        # Three equilibrium equations solve elevator, pitch and net thrust
        # scale, keeping geometry, mass and the two aerodynamic seeds fixed.
        def objective(x):
            p = [parameters[0], parameters[1], x[2]]
            return balance(p, x[0], x[1])
        sol = least_squares(objective, [targets["pitch_angle_deg"],
                                             targets["elevator_angle_deg"], parameters[2]],
                            bounds=([-6., -20., 0.2], [10., 20., 2.0]),
                            xtol=1e-11, ftol=1e-11, gtol=1e-11, max_nfev=130)
        if not sol.success or np.linalg.norm(sol.fun) > 1e-5:
            raise RuntimeError(f"Equilibrium trim failed: {sol.message}, residual={sol.fun}")
        force = bridge.evaluate([parameters[0], parameters[1], sol.x[2]],
                                sol.x[0], sol.x[1])
        return {"pitch_angle_deg": float(sol.x[0]), "elevator_angle_deg": float(sol.x[1]),
                "net_thrust_n": float(force[3]), "propeller_effective_scale": float(sol.x[2])}

    try:
        baseline = trim(params0)
        # Three physical balance equations at the two observed angles, with
        # three bounded variables. Thrust here is inferred, not FDR measured.
        # A thrust target, when provided, is a fourth, independently weighted
        # residual; measurement tolerance is NOT from CS-FSTD 2.c.5.
        def residual(x):
            f = balance(x, targets["pitch_angle_deg"], targets["elevator_angle_deg"])
            if args.target_net_thrust_n is None:
                return f
            t = bridge.evaluate(x, targets["pitch_angle_deg"],
                                targets["elevator_angle_deg"])[3]
            return np.r_[f, (t - args.target_net_thrust_n) /
                         (0.05 * args.target_net_thrust_n)]

        sol = least_squares(residual, params0,
                            bounds=([0.05, -0.25, 0.25], [0.5, 0.15, 1.75]),
                            x_scale=[0.2, 0.06, 1.],
                            xtol=1e-11, ftol=1e-11, gtol=1e-11, max_nfev=140)
        if not sol.success:
            raise RuntimeError(f"Parameter optimization failed: {sol.message}")
        optimized = trim(sol.x)
        parameters = [{"component": k.split(".")[0], "parameter": k.split(".")[1],
                       "before": float(a), "after": float(b),
                       "percent_change": float(100 * (b / a - 1))}
                      for k, a, b in zip(SEED, params0, sol.x)]
        comparisons = [{"metric": k, "fdr_target": targets[k], "before": baseline[k],
                        "after": optimized[k], "before_error": None if targets[k] is None else baseline[k]-targets[k],
                        "after_error": None if targets[k] is None else optimized[k]-targets[k],
                        "cs_fstd_2c5_tolerance_deg": TOLERANCES.get(k)}
                       for k in targets]
        report = {"status": "provisional_assumption_dependent_not_fstd_qualification",
                  "fdr_source": args.fdr.name, "fdr_sha256": fdr_sha256,
                  "relative_window_s": [START, END],
                  "n_samples": END - START + 1, "fdr_means": obs, "fdr_unit_headers": units,
                  "assumed_elevator_unit": "degrees; FDR unit header blank",
                  "pitch_trim_angle_status": "ELETRIM recorded but unit and mapping to actual tab/stabilizer angle unknown; excluded",
                  "mass_kg_assumed_not_fdr": args.mass_kg, "rho_kg_m3_proxy": args.rho,
                  "level_no_wind_assumption": "alpha = pitch; FDR AOA-2 mean retained only as diagnostic",
                  "fin_model": "fixed unvalidated proxy S=2 m2, CD0=0.009; no T6C vertical seed factory",
                  "components": ["MainWing", "HorizontalStabilizer with downwash", "DatcomFuselageComponent",
                                 "VerticalStabilizer provisional drag", "PropellerComponent scaled load"],
                  "landing_gear": "excluded; no ground-contact component installed",
                  "optimized_parameters": parameters,
                  "fixed_longitudinal_candidates": {
                      "wing": ["CL_alpha_per_rad", "CD0", "oswald_e", "Cm_alpha_per_rad", "CL_q", "Cm_q"],
                      "horizontal_tail": ["LiftCurveSlope", "ZeroLiftAngle", "ZeroLiftDragCoefficient",
                                          "InducedDragFactor", "ElevatorEffectiveness", "PitchMomentCoefficient",
                                          "PitchMomentCurveSlope", "ElevatorPitchMomentEffectiveness"],
                      "wing_tail_flow": ["referenceDownwashRad", "downwashGradientPerRad", "dynamicPressureRatio"],
                      "fuselage": ["baseDragCoefficient", "normalForceSlopePerRad",
                                   "pitchingMomentZero", "pitchingMomentSlopePerRad"],
                      "vertical_tail": ["ZeroLiftDragCoefficient (no validated T6C seed)"],
                      "propeller": ["blade pitch, chord/polar, rotation rate or calibrated thrust-vs-power map (requires engine data)"]},
                  "results": comparisons, "balance_residual_normalized": residual(sol.x).tolist(),
                  "optimizer": {"method": "scipy.optimize.least_squares", "success": bool(sol.success),
                                "message": sol.message, "nfev": sol.nfev, "bridge_evaluations": bridge.calls},
                  "limitations": ["No FDR net-thrust measurement: computed thrust is a model-inferred requirement, not a matched target.",
                                  "ELEVPOS1 unit/sign and control-to-surface mapping unverified.",
                                  "Mass, CG, vertical-tail drag and engine/propeller pitch are provisional.",
                                  "Observed pitch and AOA differ; assuming level zero-wind flight does not reproduce both.",
                                  "One snapshot cannot identify all component derivatives; remaining candidates are frozen.",
                                  "Pitch-trim angle is not modeled; its 0.5 degree tolerance cannot be checked."]}
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        with args.out.with_name(args.out.stem + "_parameters.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(parameters[0]), lineterminator="\n")
            writer.writeheader()
            writer.writerows(parameters)
        print(json.dumps({"output": str(args.out), "optimized_parameters": parameters,
                          "results": comparisons, "bridge_evaluations": bridge.calls}, indent=2))
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
