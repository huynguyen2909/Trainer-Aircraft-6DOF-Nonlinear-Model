# Aircraft configuration

The active data interface is:

```text
include/trainer_aircraft/config/T6CConfig.hpp
src/config/T6CConfig.cpp
```

`makeT6CPublicBaselineConfig()` provides only verified public manufacturer
values. `T6CConfig::isSimulationReady()` intentionally returns `false` until
flight mass, CG, inertia, wing MAC, and empennage geometry are supplied.

Do not assemble a flight model by replacing missing values with generic or
historical-aircraft defaults. Add each value to the YAML source ledger first,
including its coordinate datum and uncertainty, then expose it in the typed
configuration.

`makeT6CPC9MProxyConfig()` adds a separate Pilatus PC-9 M page-5 drawing
datum, a 1.650 m wing MAC, and an explicitly assumed CG at 30% MAC. Its
coordinates use the repository's BODY FRD convention: drawing (+X aft, +Y
right, +Z up) maps to BODY (-X, +Y, -Z). The estimated CG is
`(-0.761, 0, -2.000) m` relative to the drawing origin; its vertical position
assumes the wing's 2.000 m reference plane. See
`reference_data/t6c/geometry_mass.yaml` for the source and limitations. This
proxy factory remains incomplete and cannot assemble a simulation-ready model.
