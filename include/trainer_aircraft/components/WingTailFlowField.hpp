#pragma once

#include "trainer_aircraft/components/ILocalFlowField.hpp"

namespace trainer_aircraft
{
// Tail-averaged, quasi-steady linear downwash at one calibrated geometry/Mach.
// Angles in rad; derivatives in rad/rad. Defaults are an identity flow field,
// NOT a T-6C seed. DATCOM geometry reduction belongs in seed preparation.
struct WingTailFlowConfig
{
    double referenceBodyAlphaRad{0.0};
    double referenceDownwashRad{0.0};
    double downwashGradientPerRad{0.0};
    double referenceFlapRad{0.0};
    double flapDownwashGradientPerRad{0.0};
    double dynamicPressureRatio{1.0};
};

struct WingTailFlowEvaluation
{
    double downwashRad{0.0};
    Vec3 wakeVelocityBodyMps{}; // Before the tail's omega cross r contribution.
    Vec3 velocityIncrementBodyMps{};
};

class WingTailFlowField final : public ILocalFlowField
{
public:
    explicit WingTailFlowField(const WingTailFlowConfig& config);
    [[nodiscard]] WingTailFlowEvaluation evaluateDetailed(
        const EvaluationContext& context) const;
    [[nodiscard]] Vec3 velocityIncrementBodyMps(
        const EvaluationContext& context,
        const Vec3& positionFromCgBodyM) const override;
private:
    WingTailFlowConfig config_;
};
} // namespace trainer_aircraft
