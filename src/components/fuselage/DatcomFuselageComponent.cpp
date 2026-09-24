#include "trainer_aircraft/components/fuselage/DatcomFuselageComponent.hpp"

#include <cmath>
#include <stdexcept>

namespace trainer_aircraft::fuselage
{

DatcomFuselageComponent::DatcomFuselageComponent(DatcomFuselageConfig config)
    : config_(config)
{
    if (!(config_.referenceAreaM2 > 0.0) ||
        !(config_.referenceChordM > 0.0) ||
        !(config_.bodyLengthM > 0.0) ||
        !(config_.wettedAreaM2 > 0.0) ||
        !(config_.formFactor > 0.0) ||
        !std::isfinite(config_.baseDragCoefficient) ||
        !std::isfinite(config_.normalForceSlopePerRad) ||
        !std::isfinite(config_.pitchingMomentZero) ||
        !std::isfinite(config_.pitchingMomentSlopePerRad))
    {
        throw std::invalid_argument("Invalid DATCOM fuselage seed.");
    }
}

BodyLoad DatcomFuselageComponent::computeLoad(const EvaluationContext& context) const
{
    const auto& f = context.flightCondition;
    const auto& e = context.environment;
    if (!f.isFinite() || !e.isFinite() || f.airspeedMps < 0.0 ||
        e.airDensityKgM3 < 0.0 || e.dynamicViscosityPaS <= 0.0)
    {
        throw std::invalid_argument("Invalid fuselage flight condition.");
    }
    if (f.airspeedMps < 1.0e-10 || e.airDensityKgM3 == 0.0)
    {
        return {};
    }

    const double reynolds = e.airDensityKgM3 * f.airspeedMps *
                            config_.bodyLengthM / e.dynamicViscosityPaS;
    if (!(reynolds > 1.0))
    {
        return {};
    }
    const double cf = 0.455 / (std::pow(std::log10(reynolds), 2.58) *
                             std::pow(1.0 + 0.144 * f.mach * f.mach, 0.65));
    const double cd0 = cf * config_.formFactor * config_.wettedAreaM2 /
                       config_.referenceAreaM2 + config_.baseDragCoefficient;
    const double dynamicPressure = 0.5 * e.airDensityKgM3 *
                                   f.airspeedMps * f.airspeedMps;
    const double drag = dynamicPressure * config_.referenceAreaM2 * cd0;
    const double normal = dynamicPressure * config_.referenceAreaM2 *
                          config_.normalForceSlopePerRad * f.angleOfAttackRad;
    // Pitch coefficient is already integrated about the CG. In particular,
    // do not translate the normal force about a second arbitrary point.
    const double pitch = dynamicPressure * config_.referenceAreaM2 *
                         config_.referenceChordM *
                         (config_.pitchingMomentZero +
                          config_.pitchingMomentSlopePerRad * f.angleOfAttackRad);
    const Vec3 dragDirection = -f.airRelativeVelocityBodyMps / f.airspeedMps;
    return {drag * dragDirection + Vec3{0.0, 0.0, -normal},
            Vec3{0.0, pitch, 0.0}};
}

std::string_view DatcomFuselageComponent::name() const noexcept
{
    return "Fuselage";
}

const DatcomFuselageConfig& DatcomFuselageComponent::config() const noexcept
{
    return config_;
}

} // namespace trainer_aircraft::fuselage
