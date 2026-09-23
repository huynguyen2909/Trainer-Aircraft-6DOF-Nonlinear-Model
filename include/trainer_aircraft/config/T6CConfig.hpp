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
    std::optional<Vec3> cgFromDatumBodyM{};
    std::optional<Matrix3> inertiaAboutCgBodyKgM2{};
};

struct T6CWingGeometry
{
    double referenceAreaM2{16.28};
    double spanM{10.20};
    double aspectRatio{10.20 * 10.20 / 16.28};

    std::optional<double> meanAerodynamicChordM{};
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
    std::optional<Vec3> horizontalAerodynamicCenterFromDatumBodyM{};

    std::optional<double> verticalAreaM2{};
    std::optional<double> verticalSpanM{};
    std::optional<double> verticalMeanAerodynamicChordM{};
    std::optional<Vec3> verticalAerodynamicCenterFromDatumBodyM{};
};

struct T6CAirframeGeometry
{
    double overallLengthM{10.16};
    double overallHeightM{3.25};
    T6CWingGeometry wing{};
    T6CTailGeometry tail{};
};

struct T6CConfig
{
    T6CMassReference mass{};
    T6CAirframeGeometry geometry{};

    // True only after maneuver mass properties and the geometry needed by the
    // aerodynamic components have been populated from controlled sources.
    [[nodiscard]] bool isSimulationReady() const noexcept;
};

[[nodiscard]] T6CConfig makeT6CPublicBaselineConfig();

} // namespace trainer_aircraft
