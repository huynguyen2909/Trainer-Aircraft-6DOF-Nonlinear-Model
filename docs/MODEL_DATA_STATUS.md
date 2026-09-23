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
