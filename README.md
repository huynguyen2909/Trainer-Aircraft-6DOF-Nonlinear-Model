# Trainer Aircraft 6DOF Nonlinear Model

C++17 component-based six-degree-of-freedom simulation platform for trainer
aircraft model development and FDR-based calibration.

The `calibration/t6c-datcom-fdr-v1` branch is being converted to a T-6C model.
It currently contains a source-controlled **public baseline**, not a complete or
validated T-6C flight model. Missing mass properties and geometry are represented
as missing values; no previous-aircraft numbers are used as silent substitutes.

## Build and test

```sh
cmake -S . -B build-t6c \
  -DTRAINER_AIRCRAFT_BUILD_TESTS=ON \
  -DTRAINER_AIRCRAFT_BUILD_EXAMPLES=ON
cmake --build build-t6c --config Release
ctest --test-dir build-t6c -C Release --output-on-failure
```

On a single-config generator, `-C Release` is harmless but optional.

## Data policy

- `include/trainer_aircraft/config/T6CConfig.hpp` defines the typed baseline.
- `reference_data/t6c/geometry_mass.yaml` records values, status, units, and
  source.
- Public manufacturer masses are reference limits, not maneuver mass.
- CG, inertia, mean aerodynamic chord, empennage geometry, control limits, and
  detailed propeller geometry remain unset until controlled sources are added.
- Generic component tests use conspicuously synthetic fixtures and do not claim
  to represent T-6C performance.

See `docs/MODEL_DATA_STATUS.md` before interpreting simulation output.
