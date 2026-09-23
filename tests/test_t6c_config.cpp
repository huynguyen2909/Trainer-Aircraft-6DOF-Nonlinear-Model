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
    require(!config.mass.cgFromDrawingDatumFrdM.has_value(),
            "CG must not be guessed");
    require(!config.mass.inertiaAboutCgBodyKgM2.has_value(),
            "Inertia must not be guessed");
    require(!config.isSimulationReady(),
            "Public baseline alone must not be simulation-ready");

    const auto proxy = trainer_aircraft::makeT6CPC9MProxyConfig();
    require(!config.drawingDatumSource.has_value(),
            "Public baseline must not claim a verified drawing datum");
    require(proxy.drawingDatumSource ==
                trainer_aircraft::T6CDrawingDatumSource::PC9MModelBuildingPlanPage5,
            "Proxy must identify the auxiliary PC-9 M drawing datum");
    require(!config.geometry.wing.macLeadingEdgeFromDrawingDatumFrdM.has_value(),
            "Public baseline must not inherit proxy wing station");
    require(proxy.geometry.wing.meanAerodynamicChordM.has_value(),
            "PC-9 M proxy must specify the wing MAC");
    require(proxy.geometry.wing.macLeadingEdgeFromDrawingDatumFrdM.has_value(),
            "PC-9 M proxy must specify the MAC leading-edge station");
    require(proxy.mass.cgFromDrawingDatumFrdM.has_value(),
            "PC-9 M proxy must specify the assumed CG");
    requireNear(*proxy.geometry.wing.meanAerodynamicChordM, 1.650, 1.0e-12,
                "PC-9 M proxy wing MAC");
    const auto& macLe = *proxy.geometry.wing.macLeadingEdgeFromDrawingDatumFrdM;
    const auto& cg = *proxy.mass.cgFromDrawingDatumFrdM;
    requireNear(macLe.x, -0.266, 1.0e-12,
                "Drawing aft-positive station must be negative BODY x");
    requireNear(cg.x, macLe.x - 0.30 * *proxy.geometry.wing.meanAerodynamicChordM,
                1.0e-12, "Assumed CG must lie at 30 percent MAC in BODY FRD");
    requireNear(cg.y, 0.0, 1.0e-12, "Assumed CG must lie on the symmetry plane");
    requireNear(cg.z, -2.0, 1.0e-12,
            "Drawing up-positive reference must be negative BODY z");
    require(!config.positionFromCgBodyM(trainer_aircraft::Vec3{}).has_value(),
            "Cannot convert public geometry without an auxiliary CG datum");
    const auto cgFromCg = proxy.positionFromCgBodyM(cg);
    require(cgFromCg.has_value(), "Proxy must convert source coordinates to BODY");
    requireNear(cgFromCg->x, 0.0, 1.0e-12, "BODY x origin must be CG");
    requireNear(cgFromCg->y, 0.0, 1.0e-12, "BODY y origin must be CG");
    requireNear(cgFromCg->z, 0.0, 1.0e-12, "BODY z origin must be CG");
    const auto macLeFromCg = proxy.positionFromCgBodyM(macLe);
    require(macLeFromCg.has_value(), "Wing MAC LE must convert to BODY");
    requireNear(macLeFromCg->x, 0.495, 1.0e-12,
                "Wing MAC leading edge is 0.30 MAC ahead of assumed CG");
    requireNear(macLeFromCg->y, 0.0, 1.0e-12, "MAC LE is on symmetry plane");
    requireNear(macLeFromCg->z, 0.0, 1.0e-12,
                "MAC LE and CG share an explicitly assumed vertical level");
    require(!config.wingQuarterMacFromCgBodyM().has_value(),
            "Public baseline cannot locate the representative quarter-MAC");
    const auto quarterMacFromCg = proxy.wingQuarterMacFromCgBodyM();
    require(quarterMacFromCg.has_value(),
            "PC-9 M proxy must locate the representative quarter-MAC");
    requireNear(quarterMacFromCg->x, 0.0825, 1.0e-12,
                "Quarter-MAC must be 0.05 MAC forward of the assumed CG");
    requireNear(quarterMacFromCg->y, 0.0, 1.0e-12,
                "Representative quarter-MAC projects onto symmetry plane");
    requireNear(quarterMacFromCg->z, 0.0, 1.0e-12,
                "Quarter-MAC height shares the provisional wing level");
    require(proxy.geometry.wing.dihedralDeg.has_value(),
            "PC-9 M drawing gives the outer-wing dihedral");
    requireNear(*proxy.geometry.wing.dihedralDeg, 7.0, 1.0e-12,
                "PC-9 M outer-wing dihedral angle");
    require(!proxy.isSimulationReady(),
            "PC-9 M proxy is incomplete without mass, inertia and tail geometry");

    std::cout << "T-6C public-baseline and PC-9 M proxy configuration tests passed.\n";
    return 0;
}
