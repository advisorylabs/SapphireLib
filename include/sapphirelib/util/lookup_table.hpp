#pragma once

#include <cstddef>
#include <vector>

namespace sapphirelib {

/**
 * @brief A function of one variable, sampled at even steps and read back along a straight line
 * between samples
 *
 * For a function a hot loop calls many times, where a few millionths of error don't matter and
 * the time of an exp() or a log() does: the particle filter calls two of these per particle per
 * sensor. Reading one costs a multiply, a conversion and two loads. Pure: no PROS
 *
 * The error is at most step^2 / 8 times the largest |f''| between samples. Inputs outside
 * [minX, maxX] read the nearest end; the caller handles what lies past them
 *
 * @b Example
 * @code {.cpp}
 * // e^x from -32 to 0, 64 samples to the unit: within 3.1e-5 of exp(x), relatively
 * const sapphirelib::LookupTable expTable(-32.0, 0.0, 64, [](double x) { return std::exp(x); });
 * double half = expTable.at(-0.6931); // about 0.5
 * @endcode
 */
class LookupTable {
public:
    /**
     * @brief Sample a function, once
     *
     * @param minX the first sample's x
     * @param maxX the last sample's x
     * @param stepsPerUnit samples per unit of x. A power of two keeps every sample's x exact
     * @param function the function, called once per sample
     */
    template <typename Function>
    LookupTable(double minX, double maxX, std::size_t stepsPerUnit, Function function)
        : minX_(minX), maxX_(maxX), stepsPerUnit_(static_cast<double>(stepsPerUnit)),
          steps_(static_cast<std::size_t>((maxX - minX) * static_cast<double>(stepsPerUnit))) {
        values_.resize(steps_ + 1);
        for (std::size_t i = 0; i <= steps_; ++i) {
            values_[i] = function(minX_ + static_cast<double>(i) / stepsPerUnit_);
        }
    }

    /**
     * @brief Read the function at x, between the two samples either side
     *
     * @note always inlined: -Os, which the PROS build uses, otherwise calls it, and the call costs
     * as much as the lookup
     *
     * @param x where to read. Below minX (or NaN) reads the first sample, above maxX the last
     * @return double the interpolated value
     */
    [[gnu::always_inline]] double at(double x) const {
        const double position = (x - minX_) * stepsPerUnit_;
        if (!(position > 0.0)) return values_[0];
        if (position >= static_cast<double>(steps_)) return values_[steps_];
        const auto i = static_cast<std::size_t>(position);
        const double fraction = position - static_cast<double>(i);
        return values_[i] + fraction * (values_[i + 1] - values_[i]);
    }

    /** @brief Get the first sample's x */
    double minX() const { return minX_; }

    /** @brief Get the last sample's x */
    double maxX() const { return maxX_; }

private:
    double minX_;
    double maxX_;
    double stepsPerUnit_;
    std::size_t steps_;
    std::vector<double> values_;
};

} // namespace sapphirelib
