#include "trainer_aircraft/config/T6CWingSeed.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

int main()
{
    using namespace trainer_aircraft;
    const auto config = makeT6CWingCalibrationSeed();
    MainWing wing(config);
    RigidBodyState state;
    ControlInputs controls;
    Environment environment;
    environment.airDensityKgM3 = config.flight.rho_kg_m3;
    FlightCondition flight;
    flight.airspeedMps = config.flight.speed_m_s;
    flight.angleOfAttackRad = config.flight.alpha_deg * std::acos(-1.0) / 180.0;
    flight.airRelativeVelocityBodyMps = {
        flight.airspeedMps * std::cos(flight.angleOfAttackRad), 0.0,
        flight.airspeedMps * std::sin(flight.angleOfAttackRad)};
    flight.dynamicPressurePa = 0.5 * environment.airDensityKgM3 *
        flight.airspeedMps * flight.airspeedMps;
    flight.mach = 0.25383;
    state.velocityBodyMps = flight.airRelativeVelocityBodyMps;
    // Required context field, NOT an estimate of aircraft mass/inertia.
    // The wing load evaluation itself does not use mass properties.
    const MassProperties contextOnlyMass{1.0, Matrix3::diagonal(1.0, 1.0, 1.0)};
    const auto load = wing.evaluateDetailed(
        {0.0, state, controls, environment, flight, contextOnlyMass});
    std::cout << std::fixed << std::setprecision(9)
        << "Provisional wing-only snapshot; not trim, not an FDR fit.\n"
        << "CL=" << load.liftCoefficient << " CD=" << load.dragCoefficient
        << " Cm_CG=" << load.pitchMomentCoefficient << '\n'
        << "F_body_N=" << load.forceBodyN.x << ',' << load.forceBodyN.y
        << ',' << load.forceBodyN.z << '\n'
        << "M_CG_Nm=" << load.momentAboutCgBodyNm.x << ','
        << load.momentAboutCgBodyNm.y << ',' << load.momentAboutCgBodyNm.z << '\n';
}
