#include "trainer_aircraft/config/T6CFuselageSeed.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

int main()
{
    using namespace trainer_aircraft;
    auto fuselage = makeT6CFuselageWithDatcomSeed();
    RigidBodyState state;
    ControlInputs controls;
    Environment environment;
    environment.airDensityKgM3 = 1.10681;
    environment.dynamicViscosityPaS = 1.84e-5;
    FlightCondition flight;
    MassProperties mass{};
    const EvaluationContext context{0.0, state, controls, environment, flight, mass};
    if (fuselage->computeLoad(context).forceBodyN.norm() != 0.0)
        throw std::runtime_error("Fuselage must yield no load at zero airspeed");

    const double V=87.953;
    const double alpha=1.2292929292929293 * 3.14159265358979323846/180.0;
    flight.airspeedMps=V;
    flight.angleOfAttackRad=alpha;
    flight.airRelativeVelocityBodyMps={V*std::cos(alpha),0.0,V*std::sin(alpha)};
    flight.mach=0.25383;
    const auto plus=fuselage->computeLoad(context);
    flight.angleOfAttackRad=-alpha;
    flight.airRelativeVelocityBodyMps={V*std::cos(alpha),0.0,-V*std::sin(alpha)};
    const auto minus=fuselage->computeLoad(context);
    if (!plus.isFinite() || std::abs(plus.forceBodyN.x-minus.forceBodyN.x)>1e-8 ||
        std::abs(plus.forceBodyN.z+minus.forceBodyN.z)>1e-8 ||
        std::abs(plus.momentAboutCgBodyNm.y+minus.momentAboutCgBodyNm.y)>1e-8 ||
        !(plus.forceBodyN.z<0.0) || !(plus.momentAboutCgBodyNm.y>0.0) ||
        std::abs(plus.momentAboutCgBodyNm.y-550.0297589501607)>0.01)
    {
        throw std::runtime_error("T-6C fuselage load and CG pitch are inconsistent");
    }
    std::cout << "T-6C equivalent-body fuselage seed tests passed.\n";
}
