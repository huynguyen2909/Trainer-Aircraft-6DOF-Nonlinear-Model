#pragma once

#include "trainer_aircraft/components/fuselage/DatcomFuselageComponent.hpp"

#include <memory>

namespace trainer_aircraft
{
[[nodiscard]] fuselage::DatcomFuselageConfig makeT6CFuselageDatcomSeed();
[[nodiscard]] std::unique_ptr<fuselage::DatcomFuselageComponent>
makeT6CFuselageWithDatcomSeed();
} // namespace trainer_aircraft
