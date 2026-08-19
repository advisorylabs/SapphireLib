/**
 * \file sapphirelib/gui/odometry_page.hpp
 *
 * SapphireLib's default pose-visualization page.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>

#include "sapphirelib/gui/page.hpp"
#include "sapphirelib/odom/odometry.hpp"

namespace sapphirelib::gui {

/// Numeric x/y/heading readout plus a small live position-and-heading
/// indicator on a scaled field rectangle.
class OdometryPage : public Page {
public:
    /// `odometry` must outlive this page. `fieldWidthIn`/`fieldHeightIn`
    /// default to a 12x12ft VRC field (144in square) — override for a
    /// different game/field size.
    explicit OdometryPage(const odom::Odometry& odometry, double fieldWidthIn = 144.0,
                          double fieldHeightIn = 144.0);

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

private:
    const odom::Odometry& odometry_;
    double fieldWidthIn_;
    double fieldHeightIn_;

    lv_obj_t* poseLabel_ = nullptr;
    lv_obj_t* fieldView_ = nullptr;
    lv_obj_t* robotDot_ = nullptr;
    lv_obj_t* headingLine_ = nullptr;
    std::int32_t fieldViewWidthPx_ = 0;
    std::int32_t fieldViewHeightPx_ = 0;
};

} // namespace sapphirelib::gui
