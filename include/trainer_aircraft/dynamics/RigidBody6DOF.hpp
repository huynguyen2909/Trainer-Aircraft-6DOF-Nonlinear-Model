#pragma once

#include "trainer_aircraft/core/FlightTypes.hpp"

namespace trainer_aircraft
{

class RigidBody6DOF final
{
public:
    explicit RigidBody6DOF(const MassProperties& massProperties);

    [[nodiscard]] StateDerivative evaluate(
        const RigidBodyState& state,
        const BodyLoad& totalComponentLoad,
        const Vec3& gravityNedMps2
    ) const;

    [[nodiscard]] const MassProperties& massProperties() const noexcept;
    [[nodiscard]] const Matrix3& inverseInertiaBodyKgM2() const noexcept;

private:
    static void validateMassProperties(const MassProperties& massProperties);

    MassProperties massProperties_{};
    Matrix3 inverseInertiaBody_{};
};

} // namespace trainer_aircraft
