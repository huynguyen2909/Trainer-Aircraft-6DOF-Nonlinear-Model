#pragma once
#include "trainer_aircraft/components/WingTailFlowField.hpp"
#include "trainer_aircraft/components/stabilizers/VATC_HorizontalStabilizer.hpp"
#include <memory>
namespace trainer_aircraft {
// Provisional PC-9M/T-6C seeds, not an identified FDR model.
[[nodiscard]] HorizontalStabilizerConfig makeT6CHorizontalTailSeed();
[[nodiscard]] WingTailFlowConfig makeT6CWingTailFlowSeed();
[[nodiscard]] std::unique_ptr<HorizontalStabilizer> makeT6CHorizontalTailWithDownwash();
}
