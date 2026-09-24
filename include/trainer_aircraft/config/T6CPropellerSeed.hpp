#pragma once

#include "trainer_aircraft/components/propeller/PropellerModel.hpp"

#include <memory>

namespace trainer_aircraft
{

// PC-9M drawing geometry combined with a provisional four-blade T-6C
// section/chord/twist model. The BEMT kernel computes propulsion loads;
// DATCOM PROPWR uses the resulting THSTCP for aircraft power-on increments.
[[nodiscard]] propeller::PropellerParameters makeT6CPropellerProxyParameters();
[[nodiscard]] std::unique_ptr<propeller::PropellerComponent>
makeT6CPropellerWithProxySeed();

struct T6CDatcomPropellerPowerInputs
{
    double thrustCoefficient{0.0}; // THSTCP = T / (q_inf * Sref)
    double bladeWidthAt03RM{0.0};  // BWAPR3
    double bladeWidthAt06RM{0.0};  // BWAPR6
    double bladeWidthAt09RM{0.0};  // BWAPR9
    double bladeAngleAt075RDeg{0.0}; // BAPR75
    double radiusM{0.0};          // PRPRAD
    double hubXDrawingM{0.0};     // map to PHALOC using aircraft DATCOM datum
    double hubZDrawingM{0.0};     // map to PHVLOC using aircraft DATCOM datum
    double thrustAxisIncidenceDeg{0.0}; // AIETLP
    int engineCount{1};           // NENGSP
    int bladeCount{4};            // NOPBPE
    double hubYDrawingM{0.0};     // YP
    bool counterRotating{false};  // CROT
};

// The aerodynamic thrust is supplied by BEMT (or later by measured data).
// This mapper does not execute DATCOM or add slipstream loads to the airframe.
[[nodiscard]] T6CDatcomPropellerPowerInputs makeT6CDatcomPropellerPowerInputs(
    const propeller::PropellerParameters& parameters,
    double thrustN,
    double airDensityKgM3,
    double trueAirspeedMps,
    double referenceWingAreaM2 = 16.28);

} // namespace trainer_aircraft
