#include "trainer_aircraft/config/T6CConfig.hpp"

namespace trainer_aircraft
{

std::optional<Vec3> T6CConfig::positionFromCgBodyM(
    const Vec3& positionFromDrawingDatumFrdM) const noexcept
{
    if (!drawingDatumSource.has_value() ||
        !mass.cgFromDrawingDatumFrdM.has_value())
    {
        return std::nullopt;
    }
    return positionFromDrawingDatumFrdM - *mass.cgFromDrawingDatumFrdM;
}

bool T6CConfig::isSimulationReady() const noexcept
{
    return mass.flightMassKg.has_value() &&
        mass.cgFromDrawingDatumFrdM.has_value() &&
        mass.inertiaAboutCgBodyKgM2.has_value() &&
        geometry.wing.meanAerodynamicChordM.has_value() &&
        geometry.tail.horizontalAreaM2.has_value() &&
        geometry.tail.horizontalSpanM.has_value() &&
        geometry.tail.horizontalMeanAerodynamicChordM.has_value() &&
        geometry.tail.horizontalAerodynamicCenterFromDrawingDatumFrdM.has_value() &&
        geometry.tail.verticalAreaM2.has_value() &&
        geometry.tail.verticalSpanM.has_value() &&
        geometry.tail.verticalMeanAerodynamicChordM.has_value() &&
        geometry.tail.verticalAerodynamicCenterFromDrawingDatumFrdM.has_value();
}

T6CConfig makeT6CPublicBaselineConfig()
{
    return T6CConfig{};
}

T6CConfig makeT6CPC9MProxyConfig()
{
    auto config = makeT6CPublicBaselineConfig();

    // Pilatus PC-9 M model-building plan, page 5, in metres. Its drawing
    // coordinates are +X aft, +Y aircraft right, +Z up. BODY is FRD, so
    // (x, y, z)_FRD = (-X, +Y, -Z)_drawing for vectors from the drawing
    // origin D. The drawing's X=0 wing station, Y=0 symmetry plane and Z=0
    // reference line define D. BODY itself is always centered at CG:
    // r_(CG->P)^B = r_(D->P)^B - r_(D->CG)^B.
    constexpr double wingMacLeadingEdgeAftOfDrawingDatumM = 0.266;
    constexpr double wingMacM = 1.650;
    constexpr double assumedCgFractionOfMac = 0.30;
    constexpr double assumedCgAboveDrawingZZeroM = 2.000;

    config.drawingDatumSource =
        T6CDrawingDatumSource::PC9MModelBuildingPlanPage5;
    config.geometry.wing.meanAerodynamicChordM = wingMacM;
    // The drawing labels the MAC leading-edge X station, but gives no
    // independently verified T-6C Y/Z location. Y=0 is the symmetry plane;
    // the Z component is a provisional drawing reference for the wing plane.
    config.geometry.wing.macLeadingEdgeFromDrawingDatumFrdM =
        Vec3{-wingMacLeadingEdgeAftOfDrawingDatumM, 0.0,
             -assumedCgAboveDrawingZZeroM};
    config.mass.cgFromDrawingDatumFrdM =
        Vec3{-(wingMacLeadingEdgeAftOfDrawingDatumM +
               assumedCgFractionOfMac * wingMacM),
             0.0, -assumedCgAboveDrawingZZeroM};

    return config;
}

} // namespace trainer_aircraft
