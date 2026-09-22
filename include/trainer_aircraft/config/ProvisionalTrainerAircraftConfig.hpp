#pragma once

#include "trainer_aircraft/components/fuselage/FuselageComponent.hpp"
#include "trainer_aircraft/components/landing_gear/LandingGearComponent.hpp"
#include "trainer_aircraft/components/propeller/PropellerModel.hpp"
#include "trainer_aircraft/components/stabilizers/VATC_HorizontalStabilizer.hpp"
#include "trainer_aircraft/components/stabilizers/VATC_VerticalStabilizer.hpp"
#include "trainer_aircraft/components/wings/VATC_MainWing.hpp"
#include "trainer_aircraft/model/TrainerAircraftModel.hpp"

namespace trainer_aircraft
{

// One explicit assembly point for the current TrainerAircraft-oriented V1 estimate.
// The type name says "Provisional" because some source data are surrogates;
// see docs/MODEL_DATA_STATUS.md before interpreting performance results.
struct ProvisionalTrainerAircraftConfig
{
    MassProperties massProperties{};
    MainWingConfig mainWing{};
    HorizontalStabilizerConfig horizontalStabilizer{};
    VerticalStabilizerConfig verticalStabilizer{};
    fuselage::FuselageComponentConfig fuselage{};
    propeller::PropellerParameters propeller{};
    landing_gear::LandingGearParameters landingGear{};
};

[[nodiscard]] ProvisionalTrainerAircraftConfig makeProvisionalTrainerAircraftConfig();

// Registers five ordinary load components and the coupled Landing Gear.
// TrainerAircraftModel keeps ownership through its existing polymorphic contracts.
void addProvisionalTrainerAircraftComponents(
    TrainerAircraftModel& model,
    const ProvisionalTrainerAircraftConfig& config
);

} // namespace trainer_aircraft
