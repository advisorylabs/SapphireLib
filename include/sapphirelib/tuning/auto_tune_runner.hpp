/**
 * \file sapphirelib/tuning/auto_tune_runner.hpp
 *
 * Runs a blocking test motion while concurrently sampling a sensor signal
 * (e.g. odometry distance or heading) — the piece measureOvershoot() needs
 * that the pure math half of sapphirelib::tuning can't provide on its own.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <cstdint>
#include <functional>
#include <vector>

namespace sapphirelib::tuning {

/// Runs `motion` (expected to be a blocking call like driveDistance() or
/// turnToHeading()) on a background task while calling `sample` every
/// `samplePeriodMs` from this thread, from just before `motion` starts
/// until it returns. Blocks until `motion` completes. Returns every sample
/// collected, in order, including one taken immediately before `motion`
/// starts — feed these straight into measureOvershoot().
std::vector<double> runAndSample(const std::function<void()>& motion, const std::function<double()>& sample,
                                 std::uint32_t samplePeriodMs = 10);

} // namespace sapphirelib::tuning
