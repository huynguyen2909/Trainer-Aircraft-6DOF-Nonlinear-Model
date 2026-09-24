# Fuselage integration

The fuselage implementation is a reusable aerodynamic component. The repository
also retains `makeT6cReferenceFuselageConfig()` and its research ledger under
`reference_data/t6c/` for review.

That reference configuration is not automatically attached to the public T-6C
baseline. Its geometry, coordinate datum, approximations, and tuning constants
must be audited before it is promoted into the calibrated aircraft assembly.

An independent body-alone T-6C/PC-9M equivalent-body DATCOM seed is available
through `makeT6CFuselageWithDatcomSeed()`; it integrates fuselage normal force
and pitching moment about the proxy CG, and recomputes skin-friction drag from
the current Reynolds number. Its generator, estimated station widths/heights,
assumptions and limitations are in [the fuselage seed report](T6C_FUSELAGE_DATCOM_SEED.md).
When assembling a calibration aircraft, install **one** fuselage component;
the Nicolosi reference factory and the DATCOM seed factory are alternatives.
Neither is automatically attached to the incomplete public T-6C baseline.
