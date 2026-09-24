#include "trainer_aircraft/config/T6CPropellerSeed.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

int main()
{
    using namespace trainer_aircraft;
    constexpr double v = 87.953;
    constexpr double rho = 1.10681;
    constexpr double mach = 0.25383;
    const auto p = makeT6CPropellerProxyParameters();
    auto aircraftComponent = makeT6CPropellerWithProxySeed();
    if (aircraftComponent->model().parameters().bladeCount != 4)
        throw std::runtime_error("Aircraft-facing propeller factory missing seed");
    const auto alpha = propeller::degreesToRadians(1.2292929292929293);
    propeller::RuntimeInput flight;
    flight.airDensityKgM3 = rho;
    flight.speedOfSoundMps = v / mach;
    flight.velocityCgRelativeAirBodyMps = {v * std::cos(alpha), 0.0,
                                            v * std::sin(alpha)};
    const auto output = propeller::PropellerModel(p).evaluate(flight);
    const auto datcom = makeT6CDatcomPropellerPowerInputs(
        p, output.disk.thrustN, rho, v);

    if (p.bladeCount != 4 || std::abs(p.diameterM() - 97 * 0.0254) > 1e-12 ||
        std::abs(p.hubPositionFromCgBodyM.x - 6.3379) > 1e-9 ||
        std::abs(p.hubPositionFromCgBodyM.z + 0.4468) > 1e-9 ||
        std::abs(p.rotatingInertiaKgM2 - 27.311540731648) > 1e-9 ||
        !output.inflow.converged || !std::isfinite(output.disk.thrustN) ||
        output.disk.thrustN <= 0.0 ||
        output.disk.shaftPowerRequiredW <= 0.0 ||
        std::abs(datcom.thrustCoefficient -
                 output.disk.thrustN / (0.5 * rho * v * v * 16.28)) > 1e-12 ||
        std::abs(datcom.bladeAngleAt075RDeg - 29.0) > 1e-12 ||
        std::abs(datcom.bladeWidthAt03RM - 0.210) > 1e-12 ||
        std::abs(datcom.bladeWidthAt06RM - 0.220) > 1e-12 ||
        std::abs(datcom.bladeWidthAt09RM - 0.120) > 1e-12) {
        throw std::runtime_error("T-6C propeller proxy/PROPWR reference point inconsistent");
    }

    // A trim thrust is an input to DATCOM, independent of the blade model:
    // replacing it with FDR-calibrated thrust must change THSTCP by that ratio.
    const auto twice = makeT6CDatcomPropellerPowerInputs(
        p, 2.0 * output.disk.thrustN, rho, v);
    if (std::abs(twice.thrustCoefficient - 2.0 * datcom.thrustCoefficient) > 1e-12) {
        throw std::runtime_error("DATCOM THSTCP must follow supplied thrust");
    }
    std::cout << "T-6C propeller proxy and DATCOM power input tests passed.\n";
}
