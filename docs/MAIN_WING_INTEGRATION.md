# Main-wing integration

`VATC_MainWing` is a generic aerodynamic kernel. Its constructor validates a
fully populated `MainWingConfig`; the default configuration is intentionally
invalid because aircraft-specific geometry and coefficients must be explicit.

The current T-6C public baseline establishes only wing area, span, and derived
aspect ratio. It does not yet establish MAC, taper, sweep, incidence, flap
geometry, or aerodynamic coefficients, so it is not mapped into `MainWingConfig`.

Unit tests populate a synthetic wing fixture solely to verify equations, axes,
finite outputs, control paths, and moment transfer. Those test numbers are not
aircraft data.

## T-6C DATCOM wing initialization

The first traceable wing-only input set is
`reference_data/t6c/wing_datcom_seed.yaml`. It contains manufacturer T-6C
area/span, a clearly labelled PC-9 M MAC/outer dihedral proxy, the assumed
30%-MAC CG and the CG-origin BODY quarter-MAC point. `T6CConfig::
wingQuarterMacFromCgBodyM()` computes the latter as (+0.0825,0,0) m from the
source drawing reference: the CG is 5% MAC behind the quarter-MAC point.

In `VATC_MainWingConfig`, `reference.aero_h=0.25` and
`reference.output_h=0.30` represent those two chord fractions. The kernel
already transfers pitching moment to the output/CG reference, so an external
assembly must not apply the same wing moment arm again. The proxy's assumed
z=0 moment offset is provisional; verify the actual wing/CG separation.

The seed is intentionally not a complete Digital DATCOM deck or a wing model:
wing planform stations/taper/sweep, airfoil, incidence, signed twist, flight
grid, section polar, clean aerodynamic coefficients and flap/aileron geometry
remain unresolved. In particular the generic wing kernel validates all clean
and flap coefficients at construction; do not populate it with synthetic
fixture values under a T-6C name. DATCOM wing-only results must be assigned
to wing-only inputs, not copied from whole-aircraft derivatives.
