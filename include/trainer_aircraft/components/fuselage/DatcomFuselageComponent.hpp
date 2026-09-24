#pragma once

#include "trainer_aircraft/components/ILoadComponent.hpp"

namespace trainer_aircraft::fuselage
{

// Body-alone, small-angle equivalent-body seed. All coefficients use the
// aircraft wing area/MAC; pitching moment is already referred to the CG.
// This is deliberately separate from the historic Nicolosi component.
struct DatcomFuselageConfig
{
    double referenceAreaM2{};
    double referenceChordM{};
    double bodyLengthM{};
    double wettedAreaM2{};
    double formFactor{};
    double baseDragCoefficient{};
    double normalForceSlopePerRad{};
    double pitchingMomentZero{};
    double pitchingMomentSlopePerRad{};
};

class DatcomFuselageComponent final : public ILoadComponent
{
public:
    explicit DatcomFuselageComponent(DatcomFuselageConfig config);
    [[nodiscard]] BodyLoad computeLoad(const EvaluationContext& context) const override;
    [[nodiscard]] std::string_view name() const noexcept override;
    [[nodiscard]] const DatcomFuselageConfig& config() const noexcept;

private:
    DatcomFuselageConfig config_;
};

} // namespace trainer_aircraft::fuselage
