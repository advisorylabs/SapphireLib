#pragma once

#include <cmath>
#include <cstdint>

namespace sapphirelib {

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
 * double u = rng.uniform();  // 0 to 1
 * double n = rng.gaussian(); // mean 0, standard deviation 1
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

private:
    static std::uint32_t rotl(std::uint32_t x, int k) { return (x << k) | (x >> (32 - k)); }

    std::uint32_t state_[4] = {};
};

} // namespace sapphirelib
