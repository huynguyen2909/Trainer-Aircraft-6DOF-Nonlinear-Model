#pragma once

#include "trainer_aircraft/core/MathTypes.hpp"

#include <optional>

namespace trainer_aircraft
{

// Public manufacturer values are kept separate from flight-configuration
// values. A public basic/maximum mass must not be substituted for the actual
// mass, CG, or inertia of a particular FDR maneuver.
struct T6CMassReference
{
    double publicBasicMassKg{2336.0};
    double publicMaximumTakeoffMassKg{3765.0};
    double publicMaximumLandingMassKg{3765.0};
    double publicInternalFuelCapacityKg{544.0};

    std::optional<double> flightMassKg{};
    // Auxiliary D -> CG vector resolved along BODY FRD axes. D is the source
    // drawing datum; this is not the position of CG in the BODY frame (zero).
    std::optional<Vec3> cgFromDrawingDatumFrdM{};
    std::optional<Matrix3> inertiaAboutCgBodyKgM2{};
};

struct T6CWingGeometry
{
    double referenceAreaM2{16.28};
    double spanM{10.20};
    double aspectRatio{10.20 * 10.20 / 16.28};

    std::optional<double> meanAerodynamicChordM{};
    // Auxiliary D -> wing MAC leading-edge vector resolved along BODY FRD axes.
    // Convert to CG-origin BODY coordinates before using it as a lever arm.
    std::optional<Vec3> macLeadingEdgeFromDrawingDatumFrdM{};
    std::optional<double> rootChordM{};
    std::optional<double> tipChordM{};
    std::optional<double> taperRatio{};
    std::optional<double> leadingEdgeSweepDeg{};
    std::optional<double> incidenceDeg{};
    std::optional<double> dihedralDeg{};
};

struct T6CTailGeometry
{
    std::optional<double> horizontalAreaM2{};
    std::optional<double> horizontalSpanM{};
    std::optional<double> horizontalMeanAerodynamicChordM{};
    std::optional<Vec3> horizontalAerodynamicCenterFromDrawingDatumFrdM{};

    std::optional<double> verticalAreaM2{};
    std::optional<double> verticalSpanM{};
    std::optional<double> verticalMeanAerodynamicChordM{};
    std::optional<Vec3> verticalAerodynamicCenterFromDrawingDatumFrdM{};
};

struct T6CAirframeGeometry
{
    double overallLengthM{10.16};
    double overallHeightM{3.25};
    T6CWingGeometry wing{};
    T6CTailGeometry tail{};
};

enum class T6CDrawingDatumSource
{
    PC9MModelBuildingPlanPage5
};

struct T6CConfig
{
    // BODY is always CG-origin and FRD. The optional drawing datum D is only
    // an auxiliary origin for source measurements; its vectors use FRD axes.
    std::optional<T6CDrawingDatumSource> drawingDatumSource{};
    T6CMassReference mass{};
    T6CAirframeGeometry geometry{};

    // r_(CG->P)^B = r_(D->P)^B - r_(D->CG)^B. Returns nullopt until the
    // drawing datum and an estimated/measured CG relative to it are known.
    [[nodiscard]] std::optional<Vec3> positionFromCgBodyM(
        const Vec3& positionFromDrawingDatumFrdM) const noexcept;

    // True only after maneuver mass properties and the geometry needed by the
    // aerodynamic components have been populated from controlled sources.
    [[nodiscard]] bool isSimulationReady() const noexcept;
};

[[nodiscard]] T6CConfig makeT6CPublicBaselineConfig();

// Preliminary T-6C geometry and CG from Pilatus' PC-9 M model-building plan,
// page 5. The public T-6C dimensions remain sourced from Textron; these proxy
// values are not a measured T-6C mass distribution or a simulation-ready model.
[[nodiscard]] T6CConfig makeT6CPC9MProxyConfig();

} // namespace trainer_aircraft
