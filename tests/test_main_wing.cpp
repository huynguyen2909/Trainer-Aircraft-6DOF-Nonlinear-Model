#include "trainer_aircraft/components/wings/VATC_MainWing.hpp"
#include "trainer_aircraft/config/T6CWingSeed.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

constexpr double PI = 3.14159265358979323846;

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

struct Fixture
{
    trainer_aircraft::RigidBodyState state{};
    trainer_aircraft::ControlInputs controls{};
    trainer_aircraft::Environment environment{};
    trainer_aircraft::FlightCondition flight{};
    trainer_aircraft::MassProperties mass{
        1000.0,
        trainer_aircraft::Matrix3::diagonal(1000.0, 1500.0, 1800.0)
    };

    trainer_aircraft::EvaluationContext context() const
    {
        return {0.0, state, controls, environment, flight, mass};
    }
};

trainer_aircraft::MainWingConfig makeSyntheticWingConfig()
{
    trainer_aircraft::MainWingConfig config;
    config.wing.area_m2 = 20.0;
    config.wing.aspect_ratio = 8.0;
    config.wing.mean_chord_m = 1.5;
    config.wing.taper = 0.5;
    config.wing.sweep_le_deg = 0.0;
    config.wing.incidence_deg = 0.0;
    config.aero.cl_alpha_per_rad = 6.0;
    config.aero.CL_alpha_per_rad = 5.0;
    config.aero.CL0 = 0.1;
    config.aero.CD0 = 0.02;
    config.aero.oswald_e = 0.8;
    config.aero.Cm0 = 0.0;
    config.aero.Cm_alpha_per_rad = -0.5;
    config.flap.eta_in = 0.2;
    config.flap.eta_out = 0.6;
    config.flap.chord_ratio = 0.25;
    config.flap.thickness_ratio = 0.12;
    config.flap.section_effectiveness_per_rad = 3.0;
    config.flap.effectiveness_ratio_actual = 1.0;
    config.flap.effectiveness_ratio_reference = 1.0;
    config.flap.induced_factor_K = 0.2;
    config.lateral.Cl_da = 0.12;
    return config;
}

void testExplicitSyntheticConfiguration()
{
    trainer_aircraft::MainWing wing(makeSyntheticWingConfig());
    Fixture fixture;
    fixture.environment.airDensityKgM3 = 1.225;
    fixture.flight.airspeedMps = 50.0;
    fixture.flight.airRelativeVelocityBodyMps = {50.0, 0.0, 0.0};
    fixture.flight.dynamicPressurePa =
        0.5 * fixture.environment.airDensityKgM3 * 50.0 * 50.0;
    fixture.flight.mach = 50.0 / fixture.environment.speedOfSoundMps;
    fixture.controls.flapRad = 20.0 * PI / 180.0;

    const auto result = wing.evaluateDetailed(fixture.context());
    require(result.bodyLoad.isFinite(), "Synthetic wing result is finite");
    require(result.liftCoefficient > 0.0,
            "Positive flap deflection produces positive lift increment");
    require(result.dragCoefficient > 0.0,
            "Synthetic wing produces positive drag");
}

void testZeroSpeedAndControlPaths()
{
    trainer_aircraft::MainWing wing(makeSyntheticWingConfig());
    Fixture fixture;
    fixture.controls.flapRad = 0.0;

    const auto zero = wing.evaluateDetailed(fixture.context());
    require(zero.bodyLoad.isFinite(), "MainWing V=0 result is finite");
    requireVecNear(zero.bodyLoad.forceBodyN, {}, 0.0, "MainWing V=0 force");
    requireVecNear(zero.bodyLoad.momentAboutCgBodyNm, {}, 0.0,
                   "MainWing V=0 moment");

    fixture.flight.airspeedMps = 50.0;
    fixture.flight.airRelativeVelocityBodyMps = {50.0, 0.0, 0.0};
    fixture.flight.dynamicPressurePa =
        0.5 * fixture.environment.airDensityKgM3 * 50.0 * 50.0;
    fixture.flight.mach = 50.0 / fixture.environment.speedOfSoundMps;
    fixture.controls.aileronRad = 0.1;
    const auto aileron = wing.evaluateDetailed(fixture.context());
    require(std::abs(aileron.bodyLoad.momentAboutCgBodyNm.x) > 0.0,
            "Aileron produces wing roll moment");

    fixture.controls.flapRad = 41.0 * PI / 180.0;
    bool threw = false;
    try
    {
        (void)wing.computeLoad(fixture.context());
    }
    catch (const std::invalid_argument&)
    {
        threw = true;
    }
    require(threw, "MainWing retains 0..40 degree flap domain");
}

void testMomentTransferToCg()
{
    trainer_aircraft::MainWingConfig config = makeSyntheticWingConfig();
    config.reference.output_h = 0.50;
    config.reference.output_z_m = -0.20;
    trainer_aircraft::MainWing wing(config);

    Fixture fixture;
    fixture.flight.airspeedMps = 45.0;
    fixture.flight.airRelativeVelocityBodyMps = {44.9, 2.0, 2.2};
    fixture.flight.angleOfAttackRad = 0.049;
    fixture.flight.sideSlipRad = 0.044;
    fixture.flight.dynamicPressurePa =
        0.5 * fixture.environment.airDensityKgM3 * 45.0 * 45.0;
    fixture.flight.mach = 45.0 / fixture.environment.speedOfSoundMps;
    fixture.controls.flapRad = 15.0 * PI / 180.0;

    const auto result = wing.evaluateDetailed(fixture.context());
    const trainer_aircraft::Vec3 referenceFromCg{
        result.momentTransferDxM,
        0.0,
        result.momentTransferDzM
    };
    const trainer_aircraft::Vec3 expectedMoment =
        result.intrinsicMomentAtAerodynamicReferenceBodyNm +
        trainer_aircraft::cross(referenceFromCg, result.forceBodyN);
    requireVecNear(result.bodyLoad.momentAboutCgBodyNm, expectedMoment, 1.0e-8,
                   "MainWing transfers r cross F exactly once");
}

void testT6CSeedLoadsAndPitchRate()
{
    const auto config = trainer_aircraft::makeT6CWingCalibrationSeed();
    trainer_aircraft::MainWing wing(config);
    Fixture f;
    f.flight.airspeedMps = config.flight.speed_m_s;
    f.flight.angleOfAttackRad = config.flight.alpha_deg * PI / 180.0;
    f.environment.airDensityKgM3 = config.flight.rho_kg_m3;
    const auto steady = wing.evaluateDetailed(f.context());
    require(steady.forceBodyN.z < 0.0 && steady.dragCoefficient > 0.0,
            "Seed produces upward lift and positive drag");
    requireNear(steady.liftCoefficient, 0.339842819279, 1.0e-8,
                "Seed snapshot CL matches independently generated report");
    requireNear(steady.flapLiftCoefficientIncrement, 0.0, 0.0,
                "Split flap zero baseline");
    const trainer_aircraft::Vec3 arm{(config.reference.output_h-config.reference.aero_h)*
        config.wing.mean_chord_m, 0.0, 0.0};
    requireVecNear(steady.momentAboutCgBodyNm,
                   steady.intrinsicMomentAtAerodynamicReferenceBodyNm +
                   trainer_aircraft::cross(arm, steady.forceBodyN), 1.0e-8,
                   "Seed transfers to CG once");

    constexpr double dq = 1.0e-4;
    f.state.angularRateBodyRadps.y = dq;
    const auto plus = wing.evaluateDetailed(f.context());
    f.state.angularRateBodyRadps.y = -dq;
    const auto minus = wing.evaluateDetailed(f.context());
    const double dhat = dq*config.wing.mean_chord_m/(2*f.flight.airspeedMps);
    requireNear((plus.liftCoefficient-minus.liftCoefficient)/(2*dhat),
                config.aero.CL_q, 1.0e-8, "Pitch-rate lift derivative is active");
    requireNear((plus.pitchMomentCoefficientAtAerodynamicReference-
                 minus.pitchMomentCoefficientAtAerodynamicReference)/(2*dhat),
                config.aero.Cm_q, 1.0e-8, "Pitch-rate intrinsic damping is active");
    require(plus.momentAboutCgBodyNm.y < minus.momentAboutCgBodyNm.y,
            "Positive pitch rate adds damping about CG");

    f.state.angularRateBodyRadps = {0.1, 0.0, 0.0};
    require(wing.evaluateDetailed(f.context()).momentAboutCgBodyNm.x < 0.0,
            "Positive roll rate has wing roll damping");
    f.state.angularRateBodyRadps = {};
    f.controls.aileronRad = 5.0*PI/180.0;
    const auto right = wing.evaluateDetailed(f.context());
    f.controls.aileronRad *= -1;
    const auto left = wing.evaluateDetailed(f.context());
    require(right.momentAboutCgBodyNm.x > 0.0 && right.momentAboutCgBodyNm.z < 0.0,
            "Positive effective aileron produces right roll and adverse yaw");
    requireNear(right.momentAboutCgBodyNm.x, -left.momentAboutCgBodyNm.x, 1.0e-8,
                "Aileron antisymmetry");
    f.controls.aileronRad = 0.0;
    f.controls.flapRad = 60.0*PI/180.0;
    const auto flap = wing.evaluateDetailed(f.context());
    require(flap.bodyLoad.isFinite() && flap.liftCoefficient > steady.liftCoefficient &&
            flap.dragCoefficient > steady.dragCoefficient &&
            flap.intrinsicMomentAtAerodynamicReferenceBodyNm.y <
                steady.intrinsicMomentAtAerodynamicReferenceBodyNm.y,
            "Split flap table extends to 60 deg with lift, drag and nose-down increment");
    requireNear(flap.projectedChordRatio, 1.0, 0.0,
                "Split flap does not shorten wing chord");
    f.flight.airspeedMps = 0.0;
    f.state.angularRateBodyRadps = {0.1, 0.2, 0.3};
    const auto stopped = wing.evaluateDetailed(f.context());
    requireVecNear(stopped.forceBodyN, {}, 0.0, "Zero-speed seed force remains finite zero");
    requireVecNear(stopped.momentAboutCgBodyNm, {}, 0.0, "Zero-speed seed moment remains finite zero");
}

} // namespace

int main()
{
    testExplicitSyntheticConfiguration();
    testZeroSpeedAndControlPaths();
    testMomentTransferToCg();
    testT6CSeedLoadsAndPitchRate();
    std::cout << "MainWing tests passed.\n";
    return 0;
}
