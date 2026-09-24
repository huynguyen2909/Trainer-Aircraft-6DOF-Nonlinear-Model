#include "trainer_aircraft/config/T6CPropellerSeed.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

int main()
{
    using namespace trainer_aircraft;
    constexpr double speedMps = 87.953;
    constexpr double densityKgM3 = 1.10681;
    constexpr double alphaDeg = 1.2292929292929293;
    constexpr double mach = 0.25383;
    const double alpha = propeller::degreesToRadians(alphaDeg);
    const auto parameters = makeT6CPropellerProxyParameters();
    propeller::RuntimeInput input;
    input.velocityCgRelativeAirBodyMps = {
        speedMps * std::cos(alpha), 0.0, speedMps * std::sin(alpha)};
    input.airDensityKgM3 = densityKgM3;
    input.speedOfSoundMps = speedMps / mach;
    const auto result = propeller::PropellerModel(parameters).evaluate(input);
    const auto datcom = makeT6CDatcomPropellerPowerInputs(
        parameters, result.disk.thrustN, densityKgM3, speedMps);

    std::cout << std::setprecision(9)
              << "inflow_converged=" << result.inflow.converged << '\n'
              << "rpm=" << result.rpm << '\n'
              << "disk_area_m2=" << parameters.diskAreaM2() << '\n'
              << "advance_ratio_J=" << result.kinematics.advanceRatioJ << '\n'
              << "thrust_N=" << result.disk.thrustN << '\n'
              << "torque_Nm=" << result.disk.torqueRequiredNm << '\n'
              << "power_W=" << result.disk.shaftPowerRequiredW << '\n'
              << "CT_propeller=" << result.disk.thrustCoefficientPropeller << '\n'
              << "CQ_propeller=" << result.disk.torqueCoefficientPropeller << '\n'
              << "THSTCP_datcom=" << datcom.thrustCoefficient << '\n'
              << "power_coefficient_CP="
              << result.disk.shaftPowerRequiredW /
                     (densityKgM3 * std::pow(parameters.revolutionsPerSecond(), 3) *
                      std::pow(parameters.diameterM(), 5)) << '\n'
              << "max_section_mach=" << result.disk.maximumSectionMach << '\n'
              << "polar_clamps=" << result.disk.polarClampCount << '\n'
              << "compressibility_warning=" << result.disk.compressibilityWarning << '\n'
              << "moment_at_cg_body_y_Nm=" << result.totalMomentAtCgBodyNm.y << '\n'
              << "force_body_x_N=" << result.forceBodyN.x << '\n'
              << "force_body_y_N=" << result.forceBodyN.y << '\n'
              << "force_body_z_N=" << result.forceBodyN.z << '\n'
              << "moment_at_cg_body_x_Nm=" << result.totalMomentAtCgBodyNm.x << '\n'
              << "hub_body_x_m=" << parameters.hubPositionFromCgBodyM.x << '\n'
              << "hub_body_z_m=" << parameters.hubPositionFromCgBodyM.z << '\n';
    return result.inflow.converged ? 0 : 1;
}
