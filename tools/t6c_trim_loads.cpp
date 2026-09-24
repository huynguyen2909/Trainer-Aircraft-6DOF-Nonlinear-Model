// Numerical bridge for SciPy: evaluate the actual C++ component loads at a
// stationary, wings-level, no-wind, level-flight trial state. Input is one
// whitespace-separated record per line; output is Fx Fz My propeller Fx.
#include "trainer_aircraft/config/T6CWingSeed.hpp"
#include "trainer_aircraft/config/T6CTailSeed.hpp"
#include "trainer_aircraft/config/T6CFuselageSeed.hpp"
#include "trainer_aircraft/config/T6CPropellerSeed.hpp"
#include "trainer_aircraft/components/stabilizers/VATC_VerticalStabilizer.hpp"
#include "trainer_aircraft/model/TrainerAircraftModel.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {
class ScaledPropeller final : public trainer_aircraft::ILoadComponent {
public:
    explicit ScaledPropeller(double scale) : scale_(scale), component_(trainer_aircraft::makeT6CPropellerWithProxySeed()) {}
    trainer_aircraft::BodyLoad computeLoad(const trainer_aircraft::EvaluationContext& context) const override {
        auto load = component_->computeLoad(context);
        load.forceBodyN = scale_ * load.forceBodyN;
        load.momentAboutCgBodyNm = scale_ * load.momentAboutCgBodyNm;
        return load;
    }
    std::string_view name() const noexcept override { return "ScaledPropellerProxy"; }
private:
    double scale_;
    std::unique_ptr<trainer_aircraft::propeller::PropellerComponent> component_;
};
}

int main() {
    using namespace trainer_aircraft;
    std::cout << std::setprecision(17);
    double cl0, cm0, propScale, thetaDeg, elevatorDeg, massKg, speedMps, rho, mach, npPercent;
    while (std::cin >> cl0 >> cm0 >> propScale >> thetaDeg >> elevatorDeg >> massKg >> speedMps >> rho >> mach >> npPercent) {
        try {
            const double theta = thetaDeg * 3.14159265358979323846 / 180.0;
            const double alpha = theta; // level, no wind: gamma = theta - alpha = 0
            MassProperties mass{massKg, Matrix3::diagonal(1000., 2000., 3000.)};
            TrainerAircraftModel aircraft(mass);
            auto wing = makeT6CWingCalibrationSeed();
            wing.aero.CL0 = cl0;
            wing.aero.Cm0 = cm0;
            aircraft.addLoadComponent(std::make_unique<VATC_MainWing>(wing));
            aircraft.addLoadComponent(makeT6CHorizontalTailWithDownwash());
            aircraft.addLoadComponent(makeT6CFuselageWithDatcomSeed());
            // Explicitly unvalidated fin parasite-drag proxy; no T-6C fin seed
            // factory currently exists. Frozen throughout the optimization.
            VerticalStabilizerConfig fin;
            fin.Area = 2.0;
            fin.TailSpan = 2.0;
            fin.TailMAC = 1.0;
            fin.ZeroLiftDragCoefficient = 0.009;
            fin.PositionWrtCG[0] = -5.0;
            fin.PositionWrtCG[2] = -0.9;
            aircraft.addLoadComponent(std::make_unique<VerticalStabilizer>(fin));
            aircraft.addLoadComponent(std::make_unique<ScaledPropeller>(propScale));
            RigidBodyState state;
            state.attitudeBodyToNed = {std::cos(theta / 2.), 0., std::sin(theta / 2.), 0.};
            state.velocityBodyMps = {speedMps * std::cos(alpha), 0., speedMps * std::sin(alpha)};
            ControlInputs controls;
            controls.elevatorRad = elevatorDeg * 3.14159265358979323846 / 180.0;
            controls.propellerSpeedScale = npPercent / 100.;
            controls.landingGearExtended = false;
            Environment env;
            env.airDensityKgM3 = rho;
            env.speedOfSoundMps = speedMps / mach;
            env.dynamicViscosityPaS = 1.84e-5;
            const auto result = aircraft.evaluate(0., state, controls, env);
            if (result.groundContactCount != 0 || aircraft.hasGroundContactComponent() ||
                result.contributingComponentCount != 5) throw std::runtime_error("Component/gear mismatch");
            const double fx = massKg * result.stateDerivative.velocityRateBodyMps2.x;
            const double fz = massKg * result.stateDerivative.velocityRateBodyMps2.z;
            const double my = result.totalComponentLoad.momentAboutCgBodyNm.y;
            const double thrust = result.componentLoads.back().load.forceBodyN.x;
            std::cout << fx << ' ' << fz << ' ' << my << ' ' << thrust << '\n' << std::flush;
        } catch (const std::exception& e) {
            std::cerr << "Trim bridge error: " << e.what() << '\n';
            return 2;
        }
    }
}
