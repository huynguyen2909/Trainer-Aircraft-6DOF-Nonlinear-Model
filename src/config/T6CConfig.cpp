#include "trainer_aircraft/config/T6CConfig.hpp"

namespace trainer_aircraft
{

bool T6CConfig::isSimulationReady() const noexcept
{
    return mass.flightMassKg.has_value() &&
        mass.cgFromDatumBodyM.has_value() &&
        mass.inertiaAboutCgBodyKgM2.has_value() &&
        geometry.wing.meanAerodynamicChordM.has_value() &&
        geometry.tail.horizontalAreaM2.has_value() &&
        geometry.tail.horizontalSpanM.has_value() &&
        geometry.tail.horizontalMeanAerodynamicChordM.has_value() &&
        geometry.tail.horizontalAerodynamicCenterFromDatumBodyM.has_value() &&
        geometry.tail.verticalAreaM2.has_value() &&
        geometry.tail.verticalSpanM.has_value() &&
        geometry.tail.verticalMeanAerodynamicChordM.has_value() &&
        geometry.tail.verticalAerodynamicCenterFromDatumBodyM.has_value();
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
    // (x, y, z)_BODY = (-X, +Y, -Z)_drawing. The origin is the intersection
    // of the drawing's X=0 wing station, Y=0 symmetry plane and Z=0 reference
    // line; it is NOT the aircraft CG or a ground-fixed point.
    constexpr double wingMacLeadingEdgeAftOfDrawingDatumM = 0.266;
    constexpr double wingMacM = 1.650;
    constexpr double assumedCgFractionOfMac = 0.30;
    constexpr double assumedCgAboveDrawingZZeroM = 2.000;

    config.datumSource = T6CDatumSource::PC9MModelBuildingPlanPage5;
    config.geometry.wing.meanAerodynamicChordM = wingMacM;
    // The drawing labels the MAC leading-edge X station, but gives no
    // independently verified T-6C Y/Z location. Y=0 is the symmetry plane;
    // the Z component is a provisional drawing reference for the wing plane.
    config.geometry.wing.macLeadingEdgeFromDatumBodyM =
        Vec3{-wingMacLeadingEdgeAftOfDrawingDatumM, 0.0,
             -assumedCgAboveDrawingZZeroM};
    config.mass.cgFromDatumBodyM =
        Vec3{-(wingMacLeadingEdgeAftOfDrawingDatumM +
               assumedCgFractionOfMac * wingMacM),
             0.0, -assumedCgAboveDrawingZZeroM};

    return config;
}

} // namespace trainer_aircraft
