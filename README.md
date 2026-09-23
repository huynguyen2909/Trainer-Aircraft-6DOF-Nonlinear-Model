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

## Provisional T-6C / PC-9M main-wing seed

`makeT6CWingCalibrationSeed()` in `trainer_aircraft/config/T6CWingSeed.hpp`
provides a runnable wing-only seed at Mach 0.25383. It combines documented DATCOM
equations with explicitly labelled engineering estimates for missing inputs.
It is not an optimized model or a complete T-6C airframe configuration.

```cpp
#include "trainer_aircraft/config/T6CWingSeed.hpp"
// model is an existing TrainerAircraftModel with flight-specific mass/inertia.
model.addLoadComponent(std::make_unique<trainer_aircraft::MainWing>(
    trainer_aircraft::makeT6CWingCalibrationSeed()));
```

The detailed Vietnamese input/formula/output/reference tables are in
[`docs/T6C_MAIN_WING_DATCOM_SEED.md`](docs/T6C_MAIN_WING_DATCOM_SEED.md).
Run `python tools/generate_t6c_wing_seed.py --check` to check generated data,
or run without `--check` after changing its inputs. No Python dependency is
needed to build or run the C++ model. The `trainer_aircraft_t6c_wing_seed_demo`
target prints a static wing-only load snapshot using the new configuration.

## Wing–Tail flow framework

`WingTailFlowField` plugs into `HorizontalStabilizer` through `ILocalFlowField`.
It supplies quasi-steady downwash and a wake dynamic-pressure ratio at each evaluation.
Defaults are identity flow; T-6C tail/downwash seeds await geometry. See
[geometry checklist and integration](docs/T6C_WING_TAIL_FLOW_AND_GEOMETRY.md).
