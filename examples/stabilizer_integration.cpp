#include "trainer_aircraft/components/stabilizers/VATC_HorizontalStabilizer.hpp"
#include "trainer_aircraft/components/stabilizers/VATC_VerticalStabilizer.hpp"
#include "trainer_aircraft/model/TrainerAircraftModel.hpp"

#include <iomanip>
#include <iostream>
#include <memory>

namespace
{

constexpr double PI = 3.14159265358979323846;

double degreesToRadians(double degrees)
{
    return degrees * PI / 180.0;
}

trainer_aircraft::HorizontalStabilizerConfig makeHorizontalTailConfig()
{
    trainer_aircraft::HorizontalStabilizerConfig config;
    config.Area = 3.99483072;
    config.TailSpan = 3.99741452;
    config.TailMAC = 1.01236110;
    config.IncidenceAngle = degreesToRadians(-1.0);
    config.PositionWrtCG[0] = -4.35074096;
    config.PositionWrtCG[2] = 0.4;
    config.MomentArm = 4.35074096;
    config.ElevatorArea = 1.1;
    config.ElevatorMechanicalLimit[0] = degreesToRadians(-30.0);
    config.ElevatorMechanicalLimit[1] = degreesToRadians(20.0);
    config.LiftCurveSlope = 3.324;
    config.ZeroLiftDragCoefficient = 0.010;
    config.InducedDragFactor = 0.0884;
    config.ElevatorEffectiveness = 0.447;
    return config;
}

trainer_aircraft::VerticalStabilizerConfig makeVerticalTailConfig()
{
    constexpr double squareFootToSquareMetre = 0.09290304;

    trainer_aircraft::VerticalStabilizerConfig config;
    config.Area = 14.6 * squareFootToSquareMetre;
    config.RudderArea = 8.33 * squareFootToSquareMetre;
    config.TailSpan = 1.50; // Replace with the authoritative TrainerAircraft geometry.
    config.TailMAC = 0.90;  // Replace with the authoritative TrainerAircraft geometry.
    config.PositionWrtCG[0] = -4.649228943;
    config.PositionWrtCG[2] = -0.765145070;
    config.MomentArm = 4.649228943;
    config.RudderMechanicalLimit[0] = degreesToRadians(-23.0);
    config.RudderMechanicalLimit[1] = degreesToRadians(17.0);
    config.SideForceCurveSlope = 3.694542443;
    config.RudderEffectiveness = 0.523912357;
    config.ZeroLiftDragCoefficient = 0.012;
    config.InducedDragFactor = 0.10;
    return config;
}

} // namespace

int main()
{
    const trainer_aircraft::MassProperties massProperties{
        1250.0, // Demonstration value; replace with the selected TrainerAircraft loading.
        trainer_aircraft::Matrix3::diagonal(1800.0, 2300.0, 3500.0)
    };

    trainer_aircraft::TrainerAircraftModel model(massProperties);
    model.addLoadComponent(
        std::make_unique<trainer_aircraft::HorizontalStabilizer>(
            makeHorizontalTailConfig()
        )
    );
    model.addLoadComponent(
        std::make_unique<trainer_aircraft::VerticalStabilizer>(
            makeVerticalTailConfig()
        )
    );

    trainer_aircraft::RigidBodyState state;
    state.velocityBodyMps = {50.0, 1.0, 2.0};
    state.angularRateBodyRadps = {0.01, 0.02, -0.01};

    trainer_aircraft::ControlInputs controls;
    controls.elevatorRad = degreesToRadians(-2.0);
    controls.rudderRad = degreesToRadians(1.0);

    trainer_aircraft::Environment environment;
    const trainer_aircraft::ModelEvaluation evaluation =
        model.evaluate(0.0, state, controls, environment);

    const trainer_aircraft::BodyLoad& load = evaluation.totalComponentLoad;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "component_count  "
              << evaluation.contributingComponentCount << '\n';
    std::cout << "force_body_N     "
              << load.forceBodyN.x << ' '
              << load.forceBodyN.y << ' '
              << load.forceBodyN.z << '\n';
    std::cout << "moment_CG_body_Nm "
              << load.momentAboutCgBodyNm.x << ' '
              << load.momentAboutCgBodyNm.y << ' '
              << load.momentAboutCgBodyNm.z << '\n';
    return 0;
}
