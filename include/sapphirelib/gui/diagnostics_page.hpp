#pragma once

#include <cstdint>
#include <vector>

#include "sapphirelib/diag/sensor_check.hpp"
#include "sapphirelib/gui/gui.hpp"
#include "sapphirelib/gui/page.hpp"

namespace sapphirelib::gui {

/**
 * @brief Checks every sensor is plugged into the right port, all match long, and shows failures
 *
 * @b Example
 * @code {.cpp}
 * gui.addPage(std::make_unique<sapphirelib::gui::DiagnosticsPage>(
 *     std::vector<sapphirelib::diag::SensorCheck>{
 *         {"Front left drive", -1, sapphirelib::diag::DeviceKind::motor},
 *         {"IMU", 10, sapphirelib::diag::DeviceKind::imu},
 *     },
 *     &gui));
 * @endcode
 */
class DiagnosticsPage : public Page {
public:
    /**
     * @brief Construct a new DiagnosticsPage
     *
     * Every change is also logged as a device event (missing, lost, or back) when a telemetry
     * Logger is running, so a cable knocked loose mid-match shows up in the log
     *
     * @param checks the sensors to check
     * @param gui shows a red header banner while any check fails. nullptr (the default) for none
     */
    explicit DiagnosticsPage(std::vector<diag::SensorCheck> checks, Gui* gui = nullptr);

    const char* title() const override;
    void build(lv_obj_t* container) override;
    void update() override;

    /**
     * @brief True, so the warning banner stays honest whatever tab is showing
     *
     * The page throttles its own polling to pay for it
     */
    bool updatesWhenHidden() const override;

private:
    struct Row {
        diag::SensorCheck check;
        lv_obj_t* label = nullptr;

        // the verdict whose color is on the label, so an unchanged row isn't restyled
        bool ok = false;

        // checked at least once. Until then ok is only the starting color, not a verdict
        bool polled = false;
    };

    std::vector<Row> rows_;
    Gui* gui_;

    // pros::millis() at the last poll. 0 means never polled
    std::uint32_t lastPollMs_ = 0;
};

} // namespace sapphirelib::gui
