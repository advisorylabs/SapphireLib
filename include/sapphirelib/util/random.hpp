#pragma once

#include <cmath>
#include <cstdint>

namespace sapphirelib {

/**
 * @brief The lookup tables behind Rng::fastGaussian(): Marsaglia and Tsang's ziggurat, 128 layers
 *
 * Layer 0 is the base strip with the tail past kTailStart; layers 1 to 127 stack above it, each
 * with the same area. Built once, on the first call to zigguratTables()
 */
struct ZigguratTables {
    /** where the tail starts, in standard deviations */
    static constexpr double kTailStart = 3.442619855899;

    /** each layer's area */
    static constexpr double kLayerArea = 9.91256303526217e-3;

    /** 2^24: fastGaussian() draws a layer's x from 24 random bits */
    static constexpr double kScale = 16777216.0;

    /**
     * per layer, below which 24 bit draw the point is under the curve for certain: the next
     * narrower layer's width over this one's, times kScale
     */
    std::uint32_t k[128] = {};

    /** per layer, its width over kScale: x = draw * w */
    double w[128] = {};

    /** per layer, the curve's height exp(-x^2 / 2) at its outer edge */
    double f[128] = {};

    /**
     * @brief Build the tables (Marsaglia and Tsang 2000, with 24 bit draws)
     */
    ZigguratTables() {
        double outer = kTailStart;
        double previous = outer;
        const double q = kLayerArea / std::exp(-0.5 * outer * outer);
        k[0] = static_cast<std::uint32_t>((outer / q) * kScale);
        k[1] = 0;
        w[0] = q / kScale;
        w[127] = outer / kScale;
        f[0] = 1.0;
        f[127] = std::exp(-0.5 * outer * outer);
        for (int i = 126; i >= 1; --i) {
            outer = std::sqrt(-2.0 * std::log(kLayerArea / outer + std::exp(-0.5 * outer * outer)));
            k[i + 1] = static_cast<std::uint32_t>((outer / previous) * kScale);
            previous = outer;
            f[i] = std::exp(-0.5 * outer * outer);
            w[i] = outer / kScale;
        }
    }
};

/**
 * @brief Get the ziggurat's tables, building them on the first call
 */
inline const ZigguratTables& zigguratTables() {
    static const ZigguratTables tables;
    return tables;
}

/**
 * @brief Small, fast, seedable random number generator (xoshiro128**)
 *
 * Deterministic: the same seed gives the same sequence on the brain, on a desktop compiler, and in
 * the simulator's JavaScript port (tools/sim/js/mcl.js), which is what lets the particle filter's
 * tests pin exact results. Not for anything security related
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::Rng rng(42);
 * double u = rng.uniform();      // 0 to 1
 * double n = rng.gaussian();     // mean 0, standard deviation 1
 * double m = rng.fastGaussian(); // the same distribution, several times faster
 * @endcode
 */
class Rng {
public:
    /**
     * @brief Construct a new Rng
     *
     * @param seed any value. Each seed gives its own sequence
     */
    explicit Rng(std::uint32_t seed = 1) { reseed(seed); }

    /**
     * @brief Restart the sequence from a seed
     *
     * @param seed any value
     */
    void reseed(std::uint32_t seed) {
        // splitmix32 spreads one seed over the four state words, so nearby seeds don't give
        // nearby sequences
        std::uint32_t z = seed;
        for (std::uint32_t& word : state_) {
            z += 0x9E3779B9u;
            std::uint32_t mixed = z;
            mixed = (mixed ^ (mixed >> 16)) * 0x85EBCA6Bu;
            mixed = (mixed ^ (mixed >> 13)) * 0xC2B2AE35u;
            word = mixed ^ (mixed >> 16);
        }
    }

    /**
     * @brief Get the next raw 32 bit value
     */
    std::uint32_t next() {
        const std::uint32_t result = rotl(state_[1] * 5u, 7) * 9u;
        const std::uint32_t t = state_[1] << 9;
        state_[2] ^= state_[0];
        state_[3] ^= state_[1];
        state_[1] ^= state_[2];
        state_[0] ^= state_[3];
        state_[2] ^= t;
        state_[3] = rotl(state_[3], 11);
        return result;
    }

    /**
     * @brief Get a uniform random number
     *
     * @return double from 0 (included) to 1 (not included)
     */
    double uniform() { return static_cast<double>(next()) * (1.0 / 4294967296.0); }

    /**
     * @brief Get a normally distributed random number (Marsaglia's polar method)
     *
     * @return double mean 0, standard deviation 1
     */
    double gaussian() {
        double u = 0.0;
        double v = 0.0;
        double s = 0.0;
        do {
            u = 2.0 * uniform() - 1.0;
            v = 2.0 * uniform() - 1.0;
            s = u * u + v * v;
        } while (s >= 1.0 || s == 0.0);
        // the polar method makes a pair; the second is dropped to keep no extra state
        return u * std::sqrt(-2.0 * std::log(s) / s);
    }

    /**
     * @brief Get a normally distributed random number, by table lookup (the ziggurat method)
     *
     * The same distribution as gaussian(), but a different sequence: about 99% of draws take one
     * next(), a compare and a multiply, with no log() or sqrt(). The particle filter's predict()
     * uses it, since it draws two per particle every update
     *
     * @return double mean 0, standard deviation 1
     */
    double fastGaussian() {
        const ZigguratTables& z = zigguratTables();
        for (;;) {
            // independent bits for the layer (7), the sign (1) and the position across it (24)
            const std::uint32_t bits = next();
            const std::uint32_t layer = bits & 127u;
            const bool negative = (bits & 128u) != 0;
            const std::uint32_t across = bits >> 8;
            double x = across * z.w[layer];
            if (across < z.k[layer]) return negative ? -x : x;
            if (layer == 0) {
                // past the base strip: Marsaglia's exact tail. 1 - uniform() is never 0
                double tailX = 0.0;
                double tailY = 0.0;
                do {
                    tailX = -std::log(1.0 - uniform()) / ZigguratTables::kTailStart;
                    tailY = -std::log(1.0 - uniform());
                } while (tailY + tailY < tailX * tailX);
                x = ZigguratTables::kTailStart + tailX;
                return negative ? -x : x;
            }
            // in the wedge between the layer's rectangle and the curve: keep it if under the curve
            if (z.f[layer] + uniform() * (z.f[layer - 1] - z.f[layer]) < std::exp(-0.5 * x * x)) {
                return negative ? -x : x;
            }
        }
    }

private:
    static std::uint32_t rotl(std::uint32_t x, int k) { return (x << k) | (x >> (32 - k)); }

    std::uint32_t state_[4] = {};
};

} // namespace sapphirelib
