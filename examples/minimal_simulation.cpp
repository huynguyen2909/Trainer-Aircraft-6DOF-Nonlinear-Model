#include "trainer_aircraft/components/ILoadComponent.hpp"
#include "trainer_aircraft/integration/RK4Integrator.hpp"
#include "trainer_aircraft/model/TrainerAircraftModel.hpp"

#include <iomanip>
#include <iostream>
#include <memory>
#include <string_view>

namespace
{

class DemonstrationThrust final : public trainer_aircraft::ILoadComponent
{
public:
    trainer_aircraft::BodyLoad computeLoad(
        const trainer_aircraft::EvaluationContext& context
    ) const override
    {
        return {
            {1000.0 * context.controls.throttle, 0.0, 0.0},
            {}
        };
    }

    std::string_view name() const noexcept override
    {
        return "DemonstrationThrust";
    }
};

} // namespace

int main()
{
    const trainer_aircraft::MassProperties massProperties{
        1000.0,
        trainer_aircraft::Matrix3::diagonal(1200.0, 1500.0, 2000.0)
    };

    trainer_aircraft::TrainerAircraftModel TrainerAircraftModel(massProperties);
    TrainerAircraftModel.addLoadComponent(
        std::make_unique<DemonstrationThrust>()
    );

    trainer_aircraft::ControlInputs controls;
    controls.throttle = 1.0;

    trainer_aircraft::Environment environment;
    environment.gravityNedMps2 = {}; // Isolate the example's +X acceleration.

    trainer_aircraft::RigidBodyState state;
    trainer_aircraft::RK4Integrator integrator;

    constexpr double dtS = 0.01;
    constexpr int stepCount = 100;
    double timeS = 0.0;

    for (int step = 0; step < stepCount; ++step)
    {
        state = integrator.step(
            timeS,
            dtS,
            state,
            [&TrainerAircraftModel, &controls, &environment](
                double stageTimeS,
                const trainer_aircraft::RigidBodyState& stageState
            ) {
                return TrainerAircraftModel.evaluateDerivative(
                    stageTimeS,
                    stageState,
                    controls,
                    environment
                );
            }
        );
        timeS += dtS;
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "t_s             " << timeS << '\n';
    std::cout << "north_m         " << state.positionNedM.x << '\n';
    std::cout << "u_body_mps      " << state.velocityBodyMps.x << '\n';
    std::cout << "expected values: north_m=0.5, u_body_mps=1.0\n";
    return 0;
}

