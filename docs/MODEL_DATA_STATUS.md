# T-6C model data status

## Active public baseline

| Group | Available | Status |
|---|---|---|
| Reference mass limits | basic 2336 kg; MTOW/MLW 3765 kg; internal fuel 544 kg | Public manufacturer data; not maneuver mass |
| Overall geometry | length 10.16 m; height 3.25 m | Public manufacturer data |
| Wing | span 10.20 m; area 16.28 m²; aspect ratio 6.39066 derived as `b²/S` | Public manufacturer data plus transparent derivation |
| Flight mass, CG, inertia tensor | — | Required; not publicly established |
| Wing planform details and MAC | — | Required; not inferred from `S/b` |
| Horizontal and vertical tail geometry/positions | — | Required |
| Control-surface geometry/limits | — | Required |
| Propeller installation geometry | — | Required |

Source: Textron Aviation Defense, T-6C product page,
https://defense.txtav.com/en/t-6c (accessed 2026-09-23).

## Provisional PC-9 M drawing datum for T-6C

`makeT6CPC9MProxyConfig()` uses Pilatus' PC-9 M Model Building Plan,
PDF page 5, as a separate initial geometry hypothesis. The auxiliary drawing origin D is
the intersection of its X=0 wing station, Y=0 symmetry plane and Z=0 reference
line. BODY is CG-origin FRD, so `r_CG^B = (0, 0, 0) m`. Drawing axes (+X aft,
+Y aircraft right, +Z up) map to FRD directions `(-X, +Y, -Z)`; component
positions in BODY must also subtract the D-to-CG vector. The plan shows a
1.650 m wing MAC with its leading edge 0.266 m aft of X=0. An assumed CG at
30% MAC lies 0.761 m aft of X=0, making `r_(D->CG)^FRD=(-0.761,0,-2.000) m`
and `r_(CG->MAC_LE)^B=(+0.495,0,0) m`. The y=0 and z=-2.000 m source offsets
assume symmetry and a CG at the wing reference level; neither is a measured
T-6C CG. The document expressly restricts its drawing to model aircraft.
Source and derivation: `reference_data/t6c/geometry_mass.yaml`.

This proxy supplies neither maneuver flight mass/inertia nor complete tail and
control geometry. It remains `isSimulationReady() == false`.

The wing-only seed is now executable through `makeT6CWingCalibrationSeed()`.
`reference_data/t6c/wing_datcom_seed.yaml` records every input and its provenance,
DATCOM-derived quantities, explicit engineering proxies, and the generated
runtime configuration. It does not claim measured coefficients or an FDR fit.
The public baseline above and `makeT6CPC9MProxyConfig()` still preserve their
original source-backed dimensions; the separate calibration wing uses a
10.124 m PC-9M span and an area-normalized planform.

See [the full seed tables](T6C_MAIN_WING_DATCOM_SEED.md) for numerical values,
DATCOM page references, geometry closure, CG moment transfer, split-flap
assumptions, and frozen local lateral derivatives. The generator contains no
raw FDR records and requires only Python's standard library. Pitch-rate lift
and damping are now available in MainWing; a separate tabulated-increment path
represents the provisional split flap without applying plain-flap chord
shortening.

## Execution state

The public baseline deliberately cannot assemble a complete aircraft model.
Generic component kernels and their synthetic unit tests remain buildable. This
keeps the provisional wing calibration configuration separate from a claimed
complete or validated T-6C flight model.

## Next acceptance gate

Before enabling complete-aircraft simulation, record controlled-source values
for the missing items above, reconcile body axes and datum definitions, and add
range/consistency tests. Only then map the geometry into DATCOM inputs and add
estimated aerodynamic derivatives as a separate, traceable layer.
