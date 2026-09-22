#pragma once

#include "trainer_aircraft/components/ILoadComponent.hpp"

namespace trainer_aircraft
{

// Optional dependency for non-uniform aerodynamic flow. The returned vector
// is added to the common aircraft-relative velocity at the requested point.
// It may represent downwash, sidewash, propwash, or a local gust, but it must
// use the same BODY FRD, aircraft-relative velocity convention as FlightCondition.
class ILocalFlowField
{
public:
    virtual ~ILocalFlowField() = default;

    [[nodiscard]] virtual Vec3 velocityIncrementBodyMps(
        const EvaluationContext& context,
        const Vec3& positionFromCgBodyM
    ) const = 0;
};

} // namespace trainer_aircraft
