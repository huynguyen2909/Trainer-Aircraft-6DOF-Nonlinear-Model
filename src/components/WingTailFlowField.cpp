#include "trainer_aircraft/components/WingTailFlowField.hpp"

#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace trainer_aircraft
{
WingTailFlowField::WingTailFlowField(const WingTailFlowConfig& config)
    : config_(config)
{
    for (double value : {config.referenceBodyAlphaRad, config.referenceDownwashRad,
                         config.downwashGradientPerRad, config.referenceFlapRad,
                         config.flapDownwashGradientPerRad, config.dynamicPressureRatio})
    {
        if (!std::isfinite(value))
            throw std::invalid_argument("WingTailFlow configuration must be finite.");
    }
    if (config.dynamicPressureRatio <= 0.0)
        throw std::invalid_argument("WingTailFlow pressure ratio must be positive.");
}

WingTailFlowEvaluation WingTailFlowField::evaluateDetailed(
    const EvaluationContext& context) const
{
    const Vec3& velocity = context.flightCondition.airRelativeVelocityBodyMps;
    if (!velocity.isFinite() || !std::isfinite(context.controls.flapRad))
        throw std::invalid_argument("WingTailFlow received non-finite inputs.");
    WingTailFlowEvaluation result;
    // A zero stream has no wake velocity or well-defined angle of attack.
    if (velocity.norm() < 1.0e-8) return result;
    const double alpha = std::atan2(velocity.z, velocity.x);
    result.downwashRad = config_.referenceDownwashRad +
        config_.downwashGradientPerRad * (alpha - config_.referenceBodyAlphaRad) +
        config_.flapDownwashGradientPerRad *
            (context.controls.flapRad - config_.referenceFlapRad);
    const double c = std::cos(result.downwashRad);
    const double s = std::sin(result.downwashRad);
    const double scale = std::sqrt(config_.dynamicPressureRatio);
    // Aircraft-relative velocity, BODY FRD: positive downwash REDUCES alpha.
    // Rotate first, then scale; a pure downwash rotation preserves speed.
    result.wakeVelocityBodyMps = {
        scale * (c * velocity.x + s * velocity.z),
        scale * velocity.y,
        scale * (-s * velocity.x + c * velocity.z)
    };
    result.velocityIncrementBodyMps = result.wakeVelocityBodyMps - velocity;
    if (!std::isfinite(result.downwashRad) || !result.wakeVelocityBodyMps.isFinite() ||
        !result.velocityIncrementBodyMps.isFinite())
        throw std::runtime_error("WingTailFlow produced non-finite output.");
    return result;
}

Vec3 WingTailFlowField::velocityIncrementBodyMps(
    const EvaluationContext& context, const Vec3& positionFromCgBodyM) const
{
    if (!positionFromCgBodyM.isFinite())
        throw std::invalid_argument("WingTailFlow position must be finite.");
    // Coefficients already represent the average at the specified tail geometry.
    // Re-seed when geometry changes. Do not rotate/scale omega cross r here:
    // HorizontalStabilizer adds that term exactly once at each RK4 stage.
    return evaluateDetailed(context).velocityIncrementBodyMps;
}
} // namespace trainer_aircraft
