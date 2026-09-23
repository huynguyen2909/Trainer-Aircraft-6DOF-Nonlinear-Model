#include "trainer_aircraft/config/T6CConfig.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{

void require(bool condition, const char* message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void requireNear(double actual, double expected, double tolerance, const char* message)
{
    require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}

} // namespace

int main()
{
    const auto config = trainer_aircraft::makeT6CPublicBaselineConfig();

    requireNear(config.mass.publicBasicMassKg, 2336.0, 1.0e-12,
                "T-6C public basic mass");
    requireNear(config.mass.publicMaximumTakeoffMassKg, 3765.0, 1.0e-12,
                "T-6C public maximum takeoff mass");
    requireNear(config.mass.publicMaximumLandingMassKg, 3765.0, 1.0e-12,
                "T-6C public maximum landing mass");
    requireNear(config.mass.publicInternalFuelCapacityKg, 544.0, 1.0e-12,
                "T-6C public internal fuel capacity");
    requireNear(config.geometry.overallLengthM, 10.16, 1.0e-12,
                "T-6C overall length");
    requireNear(config.geometry.overallHeightM, 3.25, 1.0e-12,
                "T-6C overall height");
    requireNear(config.geometry.wing.referenceAreaM2, 16.28, 1.0e-12,
                "T-6C wing reference area");
    requireNear(config.geometry.wing.spanM, 10.20, 1.0e-12,
                "T-6C wing span");
    requireNear(config.geometry.wing.aspectRatio,
                10.20 * 10.20 / 16.28, 1.0e-12,
                "T-6C derived aspect ratio");

    require(!config.mass.flightMassKg.has_value(),
            "Public basic mass is not a maneuver mass");
    require(!config.mass.cgFromDatumBodyM.has_value(),
            "CG must not be guessed");
    require(!config.mass.inertiaAboutCgBodyKgM2.has_value(),
            "Inertia must not be guessed");
    require(!config.isSimulationReady(),
            "Public baseline alone must not be simulation-ready");

    std::cout << "T-6C public-baseline configuration tests passed.\n";
    return 0;
}
