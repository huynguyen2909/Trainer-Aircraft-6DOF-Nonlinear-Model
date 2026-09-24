#include "trainer_aircraft/components/ILocalFlowField.hpp"
#include "trainer_aircraft/components/WingTailFlowField.hpp"
#include <limits>
#include "trainer_aircraft/config/T6CTailSeed.hpp"
#include "trainer_aircraft/config/T6CConfig.hpp"
#include "trainer_aircraft/components/stabilizers/VATC_HorizontalStabilizer.hpp"
#include "trainer_aircraft/components/stabilizers/VATC_VerticalStabilizer.hpp"
#include "trainer_aircraft/integration/RK4Integrator.hpp"
#include "trainer_aircraft/model/TrainerAircraftModel.hpp"

#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
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

void requireNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& message
)
{
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
    {
        throw std::runtime_error(
            message + ": actual=" + std::to_string(actual) +
            ", expected=" + std::to_string(expected)
        );
    }
}

void requireVecNear(
    const trainer_aircraft::Vec3& actual,
    const trainer_aircraft::Vec3& expected,
    double tolerance,
    const std::string& message
)
{
    requireNear(actual.x, expected.x, tolerance, message + " [x]");
    requireNear(actual.y, expected.y, tolerance, message + " [y]");
    requireNear(actual.z, expected.z, tolerance, message + " [z]");
}

trainer_aircraft::HorizontalStabilizerConfig horizontalConfig()
{
    trainer_aircraft::HorizontalStabilizerConfig config;
    config.Area = 2.0;
    config.TailSpan = 4.0;
    config.TailMAC = 1.0;
    config.PositionWrtCG[0] = -4.0;
    config.ElevatorArea = 0.5;
    config.ElevatorMechanicalLimit[0] = -0.2;
    config.ElevatorMechanicalLimit[1] = 0.2;
    config.LiftCurveSlope = 1.0;
    config.ElevatorEffectiveness = 1.0;
    return config;
}

trainer_aircraft::VerticalStabilizerConfig verticalConfig()
{
    trainer_aircraft::VerticalStabilizerConfig config;
    config.Area = 2.0;
    config.RudderArea = 0.5;
    config.TailSpan = 2.0;
    config.TailMAC = 1.0;
    config.PositionWrtCG[0] = -4.0;
    config.PositionWrtCG[2] = -1.0;
    config.RudderMechanicalLimit[0] = -0.2;
    config.RudderMechanicalLimit[1] = 0.2;
    config.SideForceCurveSlope = 1.0;
    config.RudderEffectiveness = 1.0;
    return config;
}

struct ContextFixture
{
    trainer_aircraft::RigidBodyState state{};
    trainer_aircraft::ControlInputs controls{};
    trainer_aircraft::Environment environment{};
    trainer_aircraft::FlightCondition flightCondition{};
    trainer_aircraft::MassProperties massProperties{
        1000.0,
        trainer_aircraft::Matrix3::diagonal(1000.0, 1200.0, 1500.0)
    };

    trainer_aircraft::EvaluationContext context() const
    {
        return {
            0.0,
            state,
            controls,
            environment,
            flightCondition,
            massProperties
        };
    }

    void setAirRelativeVelocity(const trainer_aircraft::Vec3& velocityBodyMps)
    {
        flightCondition.airRelativeVelocityBodyMps = velocityBodyMps;
        flightCondition.airspeedMps = velocityBodyMps.norm();
        if (flightCondition.airspeedMps > 0.0)
        {
            flightCondition.angleOfAttackRad =
                std::atan2(velocityBodyMps.z, velocityBodyMps.x);
            flightCondition.sideSlipRad = std::atan2(
                velocityBodyMps.y,
                std::hypot(velocityBodyMps.x, velocityBodyMps.z)
            );
        }
        flightCondition.mach =
            flightCondition.airspeedMps / environment.speedOfSoundMps;
        flightCondition.dynamicPressurePa =
            0.5 * environment.airDensityKgM3 *
            flightCondition.airspeedMps * flightCondition.airspeedMps;
    }
};

class CountingLocalFlowField final : public trainer_aircraft::ILocalFlowField
{
public:
    explicit CountingLocalFlowField(trainer_aircraft::Vec3 incrementBodyMps)
        : incrementBodyMps_(incrementBodyMps)
    {
    }

    trainer_aircraft::Vec3 velocityIncrementBodyMps(
        const trainer_aircraft::EvaluationContext& context,
        const trainer_aircraft::Vec3& positionFromCgBodyM
    ) const override
    {
        (void)context;
        (void)positionFromCgBodyM;
        ++callCount_;
        return incrementBodyMps_;
    }

    [[nodiscard]] std::size_t callCount() const noexcept
    {
        return callCount_;
    }

private:
    trainer_aircraft::Vec3 incrementBodyMps_{};
    mutable std::size_t callCount_{0U};
};

void testHorizontalLoadAndMomentContract()
{
    trainer_aircraft::HorizontalStabilizer stabilizer(horizontalConfig());
    ContextFixture fixture;
    fixture.setAirRelativeVelocity({10.0, 0.0, 0.0});
    fixture.controls.elevatorRad = 0.1;

    const trainer_aircraft::HorizontalStabilizerEvaluation result =
        stabilizer.evaluateDetailed(fixture.context());

    constexpr double expectedDynamicPressurePa = 61.25;
    constexpr double expectedLiftN = 12.25;
    requireNear(
        result.dynamicPressurePa,
        expectedDynamicPressurePa,
        1.0e-12,
        "Horizontal-tail local dynamic pressure"
    );
    requireNear(result.liftCoefficient, 0.1, 1.0e-14, "Horizontal-tail CL");
    requireVecNear(
        result.bodyLoad.forceBodyN,
        {0.0, 0.0, -expectedLiftN},
        1.0e-12,
        "Horizontal-tail BODY force"
    );
    requireVecNear(
        result.bodyLoad.momentAboutCgBodyNm,
        {0.0, -49.0, 0.0},
        1.0e-12,
        "Horizontal-tail CG moment includes r cross F exactly once"
    );

    fixture.controls.elevatorRad = 1.0;
    const auto clamped = stabilizer.evaluateDetailed(fixture.context());
    requireNear(
        clamped.elevatorDeflectionRad,
        0.2,
        1.0e-14,
        "Elevator mechanical clamp"
    );
}

void testHorizontalAngularRateAndIntrinsicMoment()
{
    auto config = horizontalConfig();
    config.LiftCurveSlope = 0.0;
    config.PitchMomentCoefficient = 0.5;
    trainer_aircraft::HorizontalStabilizer stabilizer(config);

    ContextFixture fixture;
    fixture.setAirRelativeVelocity({10.0, 0.0, 0.0});
    fixture.state.angularRateBodyRadps = {0.0, 0.1, 0.0};

    const auto result = stabilizer.evaluateDetailed(fixture.context());
    requireVecNear(
        result.localVelocityBodyMps,
        {10.0, 0.0, 0.4},
        1.0e-14,
        "Horizontal-tail local velocity includes omega cross r"
    );
    require(
        result.intrinsicMomentAtAerodynamicCenterBodyNm.y > 0.0,
        "Positive intrinsic Cm must create positive BODY-Y moment."
    );
    requireNear(
        result.bodyLoad.momentAboutCgBodyNm.y,
        result.intrinsicMomentAtAerodynamicCenterBodyNm.y,
        1.0e-12,
        "Zero-force intrinsic pitch moment transfer"
    );
}

void testVerticalRestoringSigns()
{
    trainer_aircraft::VerticalStabilizer stabilizer(verticalConfig());
    ContextFixture fixture;
    fixture.setAirRelativeVelocity({10.0, 1.0, 0.0});

    const auto betaResult = stabilizer.evaluateDetailed(fixture.context());
    require(
        betaResult.bodyLoad.forceBodyN.y < 0.0,
        "Positive beta must produce negative BODY-Y fin force."
    );
    require(
        betaResult.bodyLoad.momentAboutCgBodyNm.z > 0.0,
        "An aft fin must produce restoring positive yaw moment for positive beta."
    );

    fixture.setAirRelativeVelocity({10.0, 0.0, 0.0});
    fixture.controls.rudderRad = 0.1; // trailing edge left
    const auto rudderResult = stabilizer.evaluateDetailed(fixture.context());
    require(
        rudderResult.bodyLoad.forceBodyN.y > 0.0,
        "Positive rudder must produce positive BODY-Y force."
    );
    require(
        rudderResult.bodyLoad.momentAboutCgBodyNm.z < 0.0,
        "Positive rudder on an aft fin must produce nose-left yaw moment."
    );
}

void testVerticalDragUsesFullLocalVelocity()
{
    auto config = verticalConfig();
    config.SideForceCurveSlope = 0.0;
    config.ZeroLiftDragCoefficient = 0.1;
    trainer_aircraft::VerticalStabilizer stabilizer(config);

    ContextFixture fixture;
    fixture.setAirRelativeVelocity({10.0, 0.0, 2.0});
    const auto result = stabilizer.evaluateDetailed(fixture.context());

    require(result.bodyLoad.forceBodyN.x < 0.0, "Fin drag must oppose +u.");
    require(
        result.bodyLoad.forceBodyN.z < 0.0,
        "Fin drag must include and oppose the local +w component."
    );
}

void testLocalFlowFieldAndRk4StageEvaluation()
{
    auto flowField =
        std::make_shared<CountingLocalFlowField>(trainer_aircraft::Vec3{0.0, 0.0, 0.5});

    trainer_aircraft::TrainerAircraftModel model({
        1000.0,
        trainer_aircraft::Matrix3::diagonal(1000.0, 1200.0, 1500.0)
    });
    model.addLoadComponent(
        std::make_unique<trainer_aircraft::HorizontalStabilizer>(
            horizontalConfig(),
            flowField
        )
    );
    model.addLoadComponent(
        std::make_unique<trainer_aircraft::VerticalStabilizer>(
            verticalConfig(),
            flowField
        )
    );

    trainer_aircraft::RigidBodyState state;
    state.velocityBodyMps = {10.0, 0.0, 0.0};
    trainer_aircraft::ControlInputs controls;
    trainer_aircraft::Environment environment;
    environment.gravityNedMps2 = {};

    trainer_aircraft::RK4Integrator integrator;
    state = integrator.step(
        0.0,
        0.01,
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

    require(state.isFinite(), "Integrated state must remain finite.");
    require(
        flowField->callCount() == 8U,
        "Two stabilizers must recompute local flow at all four RK4 stages."
    );

    const trainer_aircraft::ModelEvaluation evaluation =
        model.evaluate(0.01, state, controls, environment);
    require(
        evaluation.contributingComponentCount == 2U,
        "TrainerAircraftModel must accumulate one load from each stabilizer."
    );
    require(evaluation.totalComponentLoad.isFinite(), "Total load must be finite.");
}

void testAutomaticAspectRatioAndLimitOrdering()
{
    auto horizontal = horizontalConfig();
    horizontal.TailAR = 0.0;
    horizontal.ElevatorMechanicalLimit[0] = 0.2;
    horizontal.ElevatorMechanicalLimit[1] = -0.2;
    const trainer_aircraft::HorizontalStabilizer horizontalModel(horizontal);
    requireNear(
        horizontalModel.getConfig().TailAR,
        8.0,
        1.0e-14,
        "Horizontal-tail automatic aspect ratio"
    );
    require(
        horizontalModel.getConfig().ElevatorMechanicalLimit[0] == -0.2,
        "Elevator limit ordering"
    );

    auto vertical = verticalConfig();
    vertical.TailAR = 0.0;
    vertical.RudderMechanicalLimit[0] = 0.2;
    vertical.RudderMechanicalLimit[1] = -0.2;
    const trainer_aircraft::VerticalStabilizer verticalModel(vertical);
    requireNear(
        verticalModel.getConfig().TailAR,
        2.0,
        1.0e-14,
        "Vertical-tail automatic aspect ratio"
    );
    require(
        verticalModel.getConfig().RudderMechanicalLimit[0] == -0.2,
        "Rudder limit ordering"
    );
}


void testWingTailFlowIntegration()
{
    using namespace trainer_aircraft;
    ContextFixture f;
    auto tail = horizontalConfig();
    tail.LiftCurveSlope = 4.0;
    tail.IncidenceAngle = 0.01;
    tail.PositionWrtCG[2] = -0.4;
    f.setAirRelativeVelocity({80.0*std::cos(0.1), 0.0, 80.0*std::sin(0.1)});
    const HorizontalStabilizer plain(tail);
    const HorizontalStabilizer identity(tail, std::make_shared<WingTailFlowField>(WingTailFlowConfig{}));
    requireVecNear(identity.computeLoad(f.context()).forceBodyN,
                   plain.computeLoad(f.context()).forceBodyN, 1e-10, "Identity wake force");
    requireVecNear(identity.computeLoad(f.context()).momentAboutCgBodyNm,
                   plain.computeLoad(f.context()).momentAboutCgBodyNm, 1e-10, "Identity wake moment");
    WingTailFlowConfig c;
    c.referenceBodyAlphaRad = 0.1;
    c.referenceDownwashRad = 0.03;
    c.downwashGradientPerRad = 0.35;
    c.flapDownwashGradientPerRad = 0.1;
    c.dynamicPressureRatio = 0.9;
    auto flow = std::make_shared<WingTailFlowField>(c);
    const HorizontalStabilizer coupled(tail, flow);
    const auto r = coupled.evaluateDetailed(f.context());
    requireNear(r.flowAngleOfAttackRad, 0.07, 1e-12, "Positive downwash reduces tail alpha");
    requireNear(r.dynamicPressurePa/f.flightCondition.dynamicPressurePa, 0.9, 1e-12, "Wake pressure ratio");
    require(r.liftForceN < plain.evaluateDetailed(f.context()).liftForceN, "Downwash reduces tail lift");
    const Vec3 arm{tail.PositionWrtCG[0], 0.0, tail.PositionWrtCG[2]};
    requireVecNear(r.bodyLoad.momentAboutCgBodyNm,
        r.intrinsicMomentAtAerodynamicCenterBodyNm + cross(arm,r.bodyLoad.forceBodyN),
        1e-10, "Coupled tail transfers moment once");
    f.setAirRelativeVelocity({80.0*std::cos(0.1001), 0.0, 80.0*std::sin(0.1001)});
    const auto changed = coupled.evaluateDetailed(f.context());
    requireNear((changed.liftCoefficient-r.liftCoefficient)/0.0001,
        4.0*(1.0-0.35), 1e-9, "Tail lift slope includes downwash once");
    f.controls.flapRad = 0.2;
    requireNear(flow->evaluateDetailed(f.context()).downwashRad,
        0.03+0.35*0.0001+0.1*0.2, 1e-12, "Flap increment uses current controls");
    f.state.angularRateBodyRadps = {0.01,0.2,-0.03};
    const auto turning = coupled.evaluateDetailed(f.context());
    requireVecNear(turning.localVelocityBodyMps,
        flow->evaluateDetailed(f.context()).wakeVelocityBodyMps + cross(f.state.angularRateBodyRadps,arm),
        1e-12, "Rotation velocity added once, not scaled by wake");
    f.state.angularRateBodyRadps = {};
    c.dynamicPressureRatio = 1.0;
    const WingTailFlowField rotationOnly(c);
    requireNear(rotationOnly.evaluateDetailed(f.context()).wakeVelocityBodyMps.norm(),80.0,1e-12,
        "Pure downwash preserves speed");
    f.setAirRelativeVelocity({});
    requireVecNear(coupled.computeLoad(f.context()).forceBodyN, {}, 1e-12, "Zero speed force");
    requireVecNear(coupled.computeLoad(f.context()).momentAboutCgBodyNm, {}, 1e-12, "Zero speed moment");
    for (double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()})
    {
        c.dynamicPressureRatio = bad;
        bool rejected = false;
        try { const WingTailFlowField invalid(c); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "Reject invalid wake pressure ratio");
    }
}


void testT6CTailSeed()
{
    using namespace trainer_aircraft;
    const auto c = makeT6CHorizontalTailSeed();
    const auto proxy = makeT6CPC9MProxyConfig();
    const auto body = proxy.positionFromCgBodyM(
        *proxy.geometry.tail.horizontalAerodynamicCenterFromDrawingDatumFrdM);
    requireVecNear(*body, {c.PositionWrtCG[0],c.PositionWrtCG[1],c.PositionWrtCG[2]},
        1e-12, "Seed and proxy tail coordinates agree");
    requireNear(c.Area, *proxy.geometry.tail.horizontalAreaM2, 1e-12, "Seed and proxy area");
    requireNear(c.ElevatorArea/c.Area, 0.36, 1e-12, "Full span elevator behind 64 percent hinge");
    ContextFixture f;
    f.environment.airDensityKgM3=1.10681;
    constexpr double alpha=1.2292929292929293*3.14159265358979323846/180.0;
    f.setAirRelativeVelocity({87.953*std::cos(alpha),0.,87.953*std::sin(alpha)});
    const auto coupled = makeT6CHorizontalTailWithDownwash();
    const auto r = coupled->evaluateDetailed(f.context());
    requireNear(r.liftCoefficient,-0.15663719165575718,1e-10,"Tail seed lift snapshot");
    requireNear(r.bodyLoad.forceBodyN.z,2276.745754100379,1e-6,"Seed downforce");
    requireNear(r.bodyLoad.momentAboutCgBodyNm.y,12255.692040084796,1e-6,"Seed pitch moment including vertical drag arm");
    require(r.bodyLoad.forceBodyN.z>0. && r.bodyLoad.momentAboutCgBodyNm.y>0.,"Tail downforce creates nose-up moment");
    f.controls.elevatorRad=0.01;
    const auto deflected=coupled->evaluateDetailed(f.context());
    require(deflected.liftCoefficient>r.liftCoefficient,"Positive elevator increases lift");
    require(deflected.bodyLoad.momentAboutCgBodyNm.y<r.bodyLoad.momentAboutCgBodyNm.y,"Positive elevator decreases pitch moment");
    const auto dw=makeT6CWingTailFlowSeed();
    f.controls.elevatorRad=0.;
    f.setAirRelativeVelocity({87.953*std::cos(alpha+1e-5),0.,87.953*std::sin(alpha+1e-5)});
    requireNear((coupled->evaluateDetailed(f.context()).liftCoefficient-r.liftCoefficient)/1e-5,
        c.LiftCurveSlope*(1.-dw.downwashGradientPerRad),1e-8,"Seed downwash slope applied once");
}

} // namespace

int main()
{
    try
    {
        testT6CTailSeed();
        testWingTailFlowIntegration();
        testHorizontalLoadAndMomentContract();
        testHorizontalAngularRateAndIntrinsicMoment();
        testVerticalRestoringSigns();
        testVerticalDragUsesFullLocalVelocity();
        testLocalFlowFieldAndRk4StageEvaluation();
        testAutomaticAspectRatioAndLimitOrdering();
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }

    std::cout << "All TrainerAircraft Stage 2 stabilizer tests passed.\n";
    return 0;
}
