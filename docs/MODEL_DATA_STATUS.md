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
PDF page 5, as a separate initial geometry hypothesis. The drawing origin is
the intersection of its X=0 wing station, Y=0 symmetry plane and Z=0 reference
line. Drawing axes (+X aft, +Y aircraft right, +Z up) map to BODY FRD as
`(x, y, z) = (-X, +Y, -Z)`. The plan shows a 1.650 m wing MAC with its leading
edge 0.266 m aft of X=0. An assumed CG at 30% MAC is 0.761 m aft of X=0,
thus BODY x=-0.761 m. Its y=0 and z=-2.000 m assume symmetry and a CG on the
wing reference level 2.000 m above the drawing Z=0 line; neither assumption
is a measured T-6C CG. The document expressly restricts its drawing to model
aircraft. Source and derivation: `reference_data/t6c/geometry_mass.yaml`.

This proxy supplies neither maneuver flight mass/inertia nor complete tail and
control geometry. It remains `isSimulationReady() == false`.

## Execution state

The public baseline deliberately cannot assemble a complete aircraft model.
Generic component kernels and their synthetic unit tests remain buildable. This
prevents unverified placeholder data from contaminating DATCOM generation or FDR
calibration.

## Next acceptance gate

Before enabling complete-aircraft simulation, record controlled-source values
for the missing items above, reconcile body axes and datum definitions, and add
range/consistency tests. Only then map the geometry into DATCOM inputs and add
estimated aerodynamic derivatives as a separate, traceable layer.
