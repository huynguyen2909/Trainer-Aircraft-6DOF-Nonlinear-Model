#include "trainer_aircraft/config/ProvisionalTrainerAircraftConfig.hpp"
#include "trainer_aircraft/model/TrainerAircraftModel.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void testAllComponentsEvaluateThroughCommonArchitecture()
{
    trainer_aircraft::ProvisionalTrainerAircraftConfig aircraft =
        trainer_aircraft::makeProvisionalTrainerAircraftConfig();
    aircraft.propeller.radialElementCount = 8U;
    aircraft.propeller.azimuthStationCount = 12U;

    trainer_aircraft::TrainerAircraftModel model(aircraft.massProperties);
    trainer_aircraft::addProvisionalTrainerAircraftComponents(model, aircraft);

    require(model.componentCount() == 6U,
            "HS, VS, MainWing, Fuselage, Propeller and LandingGear registered");
    require(model.hasGroundContactComponent(),
            "LandingGear remains on the coupled ground-contact path");

    trainer_aircraft::Environment environment;
    trainer_aircraft::ControlInputs controls;
    controls.throttle = 1.0;
    controls.propellerEnabled = true;
    controls.landingGearExtended = true;
    controls.brakeLeft = 0.0;
    controls.brakeRight = 0.0;

    trainer_aircraft::RigidBodyState state;
    state.positionNedM.z = -100.0;

    const auto zeroSpeed = model.evaluate(
        0.0, 0.01, state, controls, environment
    );
    require(zeroSpeed.totalComponentLoad.isFinite(),
            "All Stage 5 components produce a finite V=0 total load");
    require(zeroSpeed.stateDerivative.isFinite(),
            "Full Stage 5 V=0 derivative is finite");
    require(zeroSpeed.groundContactCount == 0U,
            "Airborne integration fixture has no ground contact");
    require(zeroSpeed.componentLoads.size() == 6U,
            "Evaluation reports one diagnostic load per registered component");

    trainer_aircraft::BodyLoad reconstructedTotal;
    for (const auto& component : zeroSpeed.componentLoads)
    {
        require(!component.name.empty(),
                "Every diagnostic component load has a stable name");
        reconstructedTotal += component.load;
    }
    require((reconstructedTotal.forceBodyN -
             zeroSpeed.totalComponentLoad.forceBodyN).norm() < 1.0e-9,
            "Reported component forces reconstruct the accumulated force");
    require((reconstructedTotal.momentAboutCgBodyNm -
             zeroSpeed.totalComponentLoad.momentAboutCgBodyNm).norm() < 1.0e-9,
            "Reported component moments reconstruct the accumulated moment");

    state.velocityBodyMps = {40.0, 1.0, 2.0};
    state.angularRateBodyRadps = {0.02, -0.01, 0.015};
    controls.flapRad = 0.15;
    controls.aileronRad = 0.02;
    controls.elevatorRad = -0.03;
    controls.rudderRad = 0.01;

    const auto moving = model.evaluate(
        1.0, 0.01, state, controls, environment
    );
    require(moving.flightCondition.airspeedMps > 40.0,
            "TrainerAircraftModel supplies live flight condition to every component");
    require(moving.totalComponentLoad.isFinite(),
            "Full Stage 5 moving load is finite");
    require(moving.stateDerivative.isFinite(),
            "Full Stage 5 moving derivative is finite");
    require(moving.contributingComponentCount == 6U,
            "All five ordinary components plus ground-load phase were accumulated");
}

} // namespace

int main()
{
    testAllComponentsEvaluateThroughCommonArchitecture();
    std::cout << "Full-aircraft integration tests passed.\n";
    return 0;
}
