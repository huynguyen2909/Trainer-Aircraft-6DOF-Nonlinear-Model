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
