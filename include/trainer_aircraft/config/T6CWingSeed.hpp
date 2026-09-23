#pragma once

#include "trainer_aircraft/components/wings/VATC_MainWing.hpp"

namespace trainer_aircraft
{
// Provisional PC-9M geometry / NACA 4312 wing-only seed at M=0.25383.
// Mixed DATCOM equations and explicit engineering proxies; NOT an FDR fit.
// See docs/T6C_MAIN_WING_DATCOM_SEED.md. Angles passed at runtime use radians.
// Aileron input is (left-down - right-down)/2, not stick position.
[[nodiscard]] MainWingConfig makeT6CWingCalibrationSeed();
} // namespace trainer_aircraft
