#pragma once

#include "trainer_aircraft/core/FlightTypes.hpp"

#include <cstddef>

namespace trainer_aircraft
{

class LoadAccumulator final
{
public:
    void reset() noexcept;
    void add(const BodyLoad& load);

    [[nodiscard]] const BodyLoad& total() const noexcept;
    [[nodiscard]] std::size_t contributingLoadCount() const noexcept;

private:
    BodyLoad total_{};
    std::size_t contributingLoadCount_{0U};
};

} // namespace trainer_aircraft

