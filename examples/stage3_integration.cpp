#include "trainer_aircraft/components/stabilizers/VATC_HorizontalStabilizer.hpp"
#include "trainer_aircraft/components/stabilizers/VATC_VerticalStabilizer.hpp"
#include "trainer_aircraft/integration/RK4Integrator.hpp"
#include "trainer_aircraft/model/TrainerAircraftModel.hpp"
#include "trainer_aircraft/components/propeller/PropellerModel.hpp"

#include <iomanip>
#include <iostream>
#include <memory>

namespace
{

trainer_aircraft::HorizontalStabilizerConfig makeHorizontalTailConfig()
{
    trainer_aircraft::HorizontalStabilizerConfig config;
    config.Area = 3.99483072;
    config.TailSpan = 3.99741452;
    config.TailMAC = 1.01236110;
    config.PositionWrtCG[0] = -4.35074096;
    config.PositionWrtCG[2] = 0.4;
    config.ElevatorArea = 1.1;
    config.ElevatorMechanicalLimit[0] = -0.52;
    config.ElevatorMechanicalLimit[1] = 0.35;
    config.LiftCurveSlope = 3.324;
    config.ZeroLiftDragCoefficient = 0.010;
    config.InducedDragFactor = 0.0884;
    config.ElevatorEffectiveness = 0.447;
    return config;
}

trainer_aircraft::VerticalStabilizerConfig makeVerticalTailConfig()
{
    trainer_aircraft::VerticalStabilizerConfig config;
    config.Area = 1.356384384;
    config.RudderArea = 0.7738823232;
    config.TailSpan = 1.50; // Demonstration value; replace during validation.
    config.TailMAC = 0.90;  // Demonstration value; replace during validation.
    config.PositionWrtCG[0] = -4.649228943;
    config.PositionWrtCG[2] = -0.765145070;
    config.RudderMechanicalLimit[0] = -0.40;
    config.RudderMechanicalLimit[1] = 0.30;
    config.SideForceCurveSlope = 3.694542443;
    config.RudderEffectiveness = 0.523912357;
    config.ZeroLiftDragCoefficient = 0.012;
    config.InducedDragFactor = 0.10;
    return config;
}

void printVector(const char* label, const trainer_aircraft::Vec3& value)
{
    std::cout << label << ' '
              << value.x << ' ' << value.y << ' ' << value.z << '\n';
}

} // namespace

int main()
{
    // Demonstration mass properties only; replace with the validated TrainerAircraft set.
    const trainer_aircraft::MassProperties massProperties{
        1250.0,
        trainer_aircraft::Matrix3::diagonal(1800.0, 2500.0, 3200.0)
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
    model.addLoadComponent(
        std::make_unique<trainer_aircraft::propeller::PropellerComponent>(
            trainer_aircraft::propeller::makeEstimatedTrainerAircraftNaca5868_9Parameters()
        )
    );

    trainer_aircraft::RigidBodyState state;
    state.velocityBodyMps = {20.0, 0.0, 0.7};

    trainer_aircraft::ControlInputs controls;
    controls.propellerEnabled = true;
    controls.throttle = 1.0; // Reserved until an engine/governor model is added.
    controls.elevatorRad = 0.0;
    controls.rudderRad = 0.0;

    trainer_aircraft::Environment environment;
    const trainer_aircraft::ModelEvaluation evaluation =
        model.evaluate(0.0, state, controls, environment);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "component_count "
              << evaluation.contributingComponentCount << '\n';
    printVector("total_force_body_N", evaluation.totalComponentLoad.forceBodyN);
    printVector(
        "total_moment_CG_body_Nm",
        evaluation.totalComponentLoad.momentAboutCgBodyNm
    );
    printVector(
        "velocity_dot_body_mps2",
        evaluation.stateDerivative.velocityRateBodyMps2
    );

    trainer_aircraft::RK4Integrator integrator;
    const trainer_aircraft::RigidBodyState next = integrator.step(
        0.0,
        0.001,
        state,
        [&model, &controls, &environment](
            double stageTimeS,
            const trainer_aircraft::RigidBodyState& stageState
        ) {
            return model.evaluateDerivative(
                stageTimeS,
                stageState,
                controls,
                environment
            );
        }
    );
    printVector("state_velocity_after_1ms_mps", next.velocityBodyMps);
    return 0;
}
