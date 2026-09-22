# Model-data status: TrainerAircraft target versus surrogate sources

The simulated aircraft target is TrainerAircraft, but Stage 5 is still a V1 integration
model. Its software architecture and its parameter fidelity must not be
confused.

## Active runtime source

The executable does not read YAML. Its active data are constructed by
`makeProvisionalTrainerAircraftConfig()` in
`src/config/ProvisionalTrainerAircraftConfig.cpp`.

| Subsystem | Active Stage 5 data status |
| --- | --- |
| Mass/inertia | provisional integration values, not validated TrainerAircraft data |
| MainWing | TrainerAircraft-oriented geometry/defaults; lateral derivatives remain a whole-aircraft proxy |
| Horizontal/Vertical Stabilizer | TrainerAircraft-oriented estimated configuration |
| Fuselage | T-6C-derived geometry/tuning used explicitly as a provisional proxy |
| Propeller | estimated TrainerAircraft installation using the documented NACA 5868-9 surrogate |
| Landing Gear | provisional three-point geometry/stiffness used by the takeoff example |

`ProvisionalTrainerAircraftConfig` is named this way intentionally: it is the coherent
software assembly point for the TrainerAircraft-target simulation, not a claim that
all numbers have been validated for TrainerAircraft.

## Why T-6C-labelled files remain

T-6C labels now appear only where provenance requires them:

- `reference_data/t6c/`: uploaded source-value snapshots, never parsed;
- `docs/fuselage/original/`: an unchanged original-module snapshot;
- explicit `makeT6cReference...()` factories and regression tests.

Those factories remain so the uploaded Landing Gear/Fuselage results can be
reproduced. The main Stage 5 scenario does not call them by name; it calls the
provisional TrainerAircraft assembly factory.

Deleting or relabelling those reference sources as TrainerAircraft would make the data
lineage less trustworthy. Moving them outside runtime configuration makes
their actual role unambiguous.

## Next data-fidelity step

Before performance validation, replace the fields in
`makeProvisionalTrainerAircraftConfig()` with one consistent TrainerAircraft data set and add
component-level regression targets. In particular:

1. replace fuselage geometry/tuning and set its aerodynamic-reference-to-CG
   position;
2. replace MainWing whole-aircraft lateral proxy derivatives with isolated
   wing contributions, avoiding double counting with VS/Fuselage;
3. validate mass/inertia, landing-gear geometry/stiffness/friction and
   propulsion settings against the same TrainerAircraft variant;
4. only then introduce a runtime YAML/JSON schema if external configuration
   is required.
