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

    const auto proxy = trainer_aircraft::makeT6CPC9MProxyConfig();
    require(!config.datumSource.has_value(),
            "Public baseline must not claim a verified coordinate datum");
    require(proxy.datumSource ==
                trainer_aircraft::T6CDatumSource::PC9MModelBuildingPlanPage5,
            "Proxy must identify the PC-9 M drawing datum");
    require(!config.geometry.wing.macLeadingEdgeFromDatumBodyM.has_value(),
            "Public baseline must not inherit proxy wing station");
    require(proxy.geometry.wing.meanAerodynamicChordM.has_value(),
            "PC-9 M proxy must specify the wing MAC");
    require(proxy.geometry.wing.macLeadingEdgeFromDatumBodyM.has_value(),
            "PC-9 M proxy must specify the MAC leading-edge station");
    require(proxy.mass.cgFromDatumBodyM.has_value(),
            "PC-9 M proxy must specify the assumed CG");
    requireNear(*proxy.geometry.wing.meanAerodynamicChordM, 1.650, 1.0e-12,
                "PC-9 M proxy wing MAC");
    const auto& macLe = *proxy.geometry.wing.macLeadingEdgeFromDatumBodyM;
    const auto& cg = *proxy.mass.cgFromDatumBodyM;
    requireNear(macLe.x, -0.266, 1.0e-12,
                "Drawing aft-positive station must be negative BODY x");
    requireNear(cg.x, macLe.x - 0.30 * *proxy.geometry.wing.meanAerodynamicChordM,
                1.0e-12, "Assumed CG must lie at 30 percent MAC in BODY FRD");
    requireNear(cg.y, 0.0, 1.0e-12, "Assumed CG must lie on the symmetry plane");
    requireNear(cg.z, -2.0, 1.0e-12,
                "Drawing up-positive reference must be negative BODY z");
    require(!proxy.isSimulationReady(),
            "PC-9 M proxy is incomplete without mass, inertia and tail geometry");

    std::cout << "T-6C public-baseline and PC-9 M proxy configuration tests passed.\n";
    return 0;
}
