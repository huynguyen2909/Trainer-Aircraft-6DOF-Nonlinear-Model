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
