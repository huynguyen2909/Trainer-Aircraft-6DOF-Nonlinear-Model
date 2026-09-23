#pragma once

#include "trainer_aircraft/components/ILoadComponent.hpp"
#include "trainer_aircraft/components/fuselage/FuselageAerodynamics.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace trainer_aircraft::fuselage
{

// The uploaded scalar kernel deliberately leaves drag-direction resolution to
// its caller. This enum makes that retained caller choice explicit.
enum class DragDirectionMode
{
    OppositeAirRelativeVelocity,
    BodyNegativeX
};

struct FuselageComponentConfig
{
    FuselageGeometry geometry{};
    FuselageTuning tuning{};

    // Position of the paper's aerodynamic reference point (x/Lf = 0.50,
    // z/df = 0.50) relative to the aircraft CG, expressed in BODY FRD axes.
    Vec3 aerodynamicReferencePositionFromCgBodyM{};

    DragDirectionMode dragDirectionMode{
        DragDirectionMode::OppositeAirRelativeVelocity
    };
};

struct FuselageEvaluation
{
    BodyLoad bodyLoad{};
    FuselageAeroState aerodynamicState{};
    FuselageAerodynamicOutput aerodynamicOutput{};

    Vec3 unitDragDirectionBody{};
    Vec3 dragForceBodyN{};
    Vec3 intrinsicMomentAtReferenceBodyNm{};
    bool zeroDynamicPressureBypass{false};
    std::vector<std::string> warnings{};
};

class FuselageComponent final : public ILoadComponent
{
public:
    explicit FuselageComponent(FuselageComponentConfig config);

    [[nodiscard]] BodyLoad computeLoad(
        const EvaluationContext& context
    ) const override;

    [[nodiscard]] std::string_view name() const noexcept override;

    [[nodiscard]] FuselageEvaluation evaluateDetailed(
        const EvaluationContext& context
    ) const;

    [[nodiscard]] const FuselageComponentConfig& config() const noexcept;

private:
    FuselageComponentConfig config_;
    FuselageAerodynamics model_;
};

[[nodiscard]] FuselageGeometry
makeNicolosiReferenceFuselageGeometry();

[[nodiscard]] FuselageGeometry makeT6cReferenceFuselageGeometry();
[[nodiscard]] FuselageTuning makeT6cReferenceFuselageTuning();
[[nodiscard]] FuselageComponentConfig makeT6cReferenceFuselageConfig();

} // namespace trainer_aircraft::fuselage
