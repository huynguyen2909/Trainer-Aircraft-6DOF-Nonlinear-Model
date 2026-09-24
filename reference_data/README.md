# Reference data

Reference data are kept separate from executable component defaults.

- `t6c/geometry_mass.yaml`: public T-6C mass and external geometry baseline.
- `t6c/fuselage_source_parameters.yaml`: earlier T-6C fuselage research data;
  review provenance and uncertainty before promoting any value.
- `t6c/landing_gear_source_parameters.yaml`: earlier landing-gear research data;
  not part of the active mass/geometry baseline.
- `t6c/propeller_datcom_seed.yaml`: opt-in PC-9M/T-6C propeller proxy,
  reference-point BEMT estimates, and DATCOM PROPWR inputs; see
  `docs/T6C_PROPELLER_DATCOM_SEED.md` for uncertainty and missing power effects.

A value becomes active only after its source, units, reference datum, sign
convention, configuration, and confidence have been reviewed.
