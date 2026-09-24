#include "trainer_aircraft/config/T6CPropellerSeed.hpp"

#include "trainer_aircraft/config/T6CConfig.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace trainer_aircraft
{
namespace
{
constexpr double kRadiusM = 97.0 * 0.0254 / 2.0;
constexpr double kBladeAngleAt075RDeg = 29.0;
constexpr double kApproximateCompletePropellerMassKg = 70.3;
constexpr double kProvisionalBladeMassFraction = 0.60;
constexpr double kRootCutoutFraction = 0.20;

double pitchAtRadiusFraction(double radiusFraction)
{
    // Constant geometric helical pitch; collective is frozen at the chosen
    // reference setting until an engine/prop governor is available.
    const double referencePitch =
        propeller::degreesToRadians(kBladeAngleAt075RDeg);
    return std::atan(0.75 * std::tan(referencePitch) / radiusFraction);
}

propeller::BladeStation interpolateBlade(
    double radiusFraction, const propeller::PropellerParameters& p)
{
    const auto& stations = p.bladeGeometry;
    for (std::size_t i = 1; i < stations.size(); ++i) {
        if (radiusFraction <= stations[i].radiusFraction) {
            const auto& a = stations[i - 1];
            const auto& b = stations[i];
            const double u = (radiusFraction - a.radiusFraction) /
                (b.radiusFraction - a.radiusFraction);
            return {radiusFraction,
                    a.chordM + u * (b.chordM - a.chordM),
                    a.pitchRad + u * (b.pitchRad - a.pitchRad)};
        }
    }
    return stations.back();
}

} // namespace

propeller::PropellerParameters makeT6CPropellerProxyParameters()
{
    propeller::PropellerParameters p;
    p.propellerName = "T-6C Hartzell 97 in / PC-9M provisional proxy";
    p.airfoilName = "NACA 16-series-like analytic proxy (unmeasured polar)";
    p.bladeCount = 4;
    p.radiusM = kRadiusM;
    p.rootCutoutFraction = kRootCutoutFraction; // estimated spinner/root cutout
    p.rotationRateRadps = 2000.0 * 2.0 * propeller::pi / 60.0;
    p.rotationSign = +1; // assumed clockwise, seen from cockpit looking ahead
    // EASA IM.P.133: approximate maximum complete weight 70.3 kg for
    // HC-E4A-2/E9612, but no rotating inertia. Approximate 60% of mass as
    // blades distributed uniformly between r0 and R; the rest is a disk at
    // radius r0. This is an explicitly provisional gyro seed, not a TCDS Ixx.
    const double bladeMass = kApproximateCompletePropellerMassKg *
        kProvisionalBladeMassFraction;
    const double hubMass = kApproximateCompletePropellerMassKg - bladeMass;
    p.rotatingInertiaKgM2 =
        bladeMass * (kRootCutoutFraction * kRootCutoutFraction +
                     kRootCutoutFraction + 1.0) / 3.0 * p.radiusM * p.radiusM +
        0.5 * hubMass * kRootCutoutFraction * kRootCutoutFraction *
            p.radiusM * p.radiusM;
    // Drawing: hub X=-1.9704 m, Z=+2.4468 m. CG X=4.3675 m,
    // Z=+2.000 m is the existing 30%-MAC proxy (drawing +X aft/+Z up).
    const auto configuration = makeT6CPC9MProxyConfig();
    const auto hubPosition = configuration.positionFromCgBodyM(
        {+1.9704, 0.0, -2.4468});
    if (!hubPosition.has_value()) {
        throw std::logic_error("PC-9M proxy drawing datum/CG unavailable");
    }
    p.hubPositionFromCgBodyM = *hubPosition;
    p.bodyFromPropeller = Matrix3::identity(); // shaft incidence: 0 deg proxy

    // Four-blade planform cannot be resolved into section chords from page 5;
    // station widths below are explicitly provisional, not Hartzell data.
    for (const auto& station : std::vector<std::pair<double, double>>{
             {0.20, 0.170}, {0.30, 0.210}, {0.45, 0.240},
             {0.60, 0.220}, {0.75, 0.190}, {0.90, 0.120},
             {1.00, 0.045}}) {
        p.bladeGeometry.push_back({station.first, station.second,
                                   pitchAtRadiusFraction(station.first)});
    }

    // Analytic low-alpha section proxy. A genuine NACA 16-series measured
    // polar is NOT inferred from its name; Mach/Re variations are not modeled.
    std::vector<propeller::PolarPoint> polar;
    for (int alphaDeg = -24; alphaDeg <= 24; alphaDeg += 2) {
        const double alphaRad = propeller::degreesToRadians(
            static_cast<double>(alphaDeg));
        const double cl = std::clamp(
            5.7 * (alphaRad + propeller::degreesToRadians(2.0)),
            -1.15, 1.15);
        polar.push_back({alphaRad, cl, 0.012 + 0.020 * cl * cl});
    }
    p.airfoilPolar = propeller::AirfoilPolar(std::move(polar));
    p.radialElementCount = 48U;
    p.azimuthStationCount = 48U;
    p.inflow.lambdaMaximum = 1.0;
    p.inflow.bracketScanIntervals = 120U;
    p.validate();
    return p;
}

std::unique_ptr<propeller::PropellerComponent> makeT6CPropellerWithProxySeed()
{
    return std::make_unique<propeller::PropellerComponent>(
        makeT6CPropellerProxyParameters());
}

T6CDatcomPropellerPowerInputs makeT6CDatcomPropellerPowerInputs(
    const propeller::PropellerParameters& p,
    double thrustN,
    double airDensityKgM3,
    double trueAirspeedMps,
    double referenceWingAreaM2)
{
    p.validate();
    if (!std::isfinite(thrustN) || !std::isfinite(airDensityKgM3) ||
        !std::isfinite(trueAirspeedMps) ||
        !std::isfinite(referenceWingAreaM2) ||
        !(airDensityKgM3 > 0.0) || !(trueAirspeedMps > 0.0) ||
        !(referenceWingAreaM2 > 0.0)) {
        throw std::invalid_argument("DATCOM PROPWR requires finite positive flight references");
    }
    T6CDatcomPropellerPowerInputs input;
    input.thrustCoefficient = thrustN /
        (0.5 * airDensityKgM3 * trueAirspeedMps * trueAirspeedMps *
         referenceWingAreaM2);
    input.bladeWidthAt03RM = interpolateBlade(0.3, p).chordM;
    input.bladeWidthAt06RM = interpolateBlade(0.6, p).chordM;
    input.bladeWidthAt09RM = interpolateBlade(0.9, p).chordM;
    input.bladeAngleAt075RDeg =
        interpolateBlade(0.75, p).pitchRad * 180.0 / propeller::pi;
    input.radiusM = p.radiusM;
    input.hubXDrawingM = -1.9704;
    input.hubZDrawingM = 2.4468;
    input.engineCount = 1;
    input.bladeCount = p.bladeCount;
    return input;
}

} // namespace trainer_aircraft
