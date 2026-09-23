#include "trainer_aircraft/config/T6CConfig.hpp"

namespace trainer_aircraft
{

bool T6CConfig::isSimulationReady() const noexcept
{
    return mass.flightMassKg.has_value() &&
        mass.cgFromDatumBodyM.has_value() &&
        mass.inertiaAboutCgBodyKgM2.has_value() &&
        geometry.wing.meanAerodynamicChordM.has_value() &&
        geometry.tail.horizontalAreaM2.has_value() &&
        geometry.tail.horizontalSpanM.has_value() &&
        geometry.tail.horizontalMeanAerodynamicChordM.has_value() &&
        geometry.tail.horizontalAerodynamicCenterFromDatumBodyM.has_value() &&
        geometry.tail.verticalAreaM2.has_value() &&
        geometry.tail.verticalSpanM.has_value() &&
        geometry.tail.verticalMeanAerodynamicChordM.has_value() &&
        geometry.tail.verticalAerodynamicCenterFromDatumBodyM.has_value();
}

T6CConfig makeT6CPublicBaselineConfig()
{
    return T6CConfig{};
}

} // namespace trainer_aircraft
