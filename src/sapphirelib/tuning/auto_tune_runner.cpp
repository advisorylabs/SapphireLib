#include "sapphirelib/tuning/auto_tune_runner.hpp"

#include <atomic>

#include "pros/rtos.hpp"

namespace sapphirelib::tuning {

std::vector<double> runAndSample(const std::function<void()>& motion, const std::function<double()>& sample,
                                 std::uint32_t samplePeriodMs) {
    std::atomic<bool> done{false};

    // Safe to capture motion/done by reference: this function blocks below
    // until the task signals done, so its stack frame — and everything on
    // it — outlives the task.
    pros::Task task([&motion, &done] {
        motion();
        done.store(true);
    });

    std::vector<double> samples;
    samples.push_back(sample());
    while (!done.load()) {
        pros::delay(samplePeriodMs);
        samples.push_back(sample());
    }
    // One last sample right after completion, in case the motion settled
    // between the final periodic sample and done becoming visible here.
    samples.push_back(sample());

    return samples;
}

} // namespace sapphirelib::tuning
