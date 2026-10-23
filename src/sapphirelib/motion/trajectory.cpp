#include "sapphirelib/motion/trajectory.hpp"

#include <algorithm>
#include <utility>

namespace sapphirelib::motion {

namespace {

double lerp(double a, double b, double f) { return a + (b - a) * f; }

} // namespace

Trajectory::Trajectory(std::vector<TrajectorySample> samples,
                       std::vector<SectionStart> sectionStarts)
    : samples_(std::move(samples)), sectionStarts_(std::move(sectionStarts)) {
    if (sectionStarts_.empty()) sectionStarts_.push_back(SectionStart{});
}

bool Trajectory::empty() const { return samples_.empty(); }

double Trajectory::durationS() const { return samples_.empty() ? 0.0 : samples_.back().tS; }

double Trajectory::lengthIn() const { return samples_.empty() ? 0.0 : samples_.back().sIn; }

std::size_t Trajectory::sectionCount() const {
    return samples_.empty() ? 0 : sectionStarts_.size();
}

SectionStart Trajectory::sectionStart(std::size_t section) const {
    if (section < sectionStarts_.size()) return sectionStarts_[section];
    return SectionStart{.tS = durationS(), .sIn = lengthIn()};
}

std::size_t Trajectory::sectionAt(double sIn) const {
    // a section starts at its first point: the last start at or before sIn
    std::size_t section = 0;
    while (section + 1 < sectionStarts_.size() && sectionStarts_[section + 1].sIn <= sIn) {
        ++section;
    }
    return section;
}

TrajectorySample Trajectory::sample(double tS) const {
    if (samples_.empty()) return TrajectorySample{};
    if (!(tS > samples_.front().tS)) return samples_.front();
    if (tS >= samples_.back().tS) return samples_.back();

    // the first sample after tS, and the one before it
    const auto after =
        std::upper_bound(samples_.begin(), samples_.end(), tS,
                         [](double t, const TrajectorySample& sample) { return t < sample.tS; });
    const TrajectorySample& b = *after;
    const TrajectorySample& a = *(after - 1);
    const double span = b.tS - a.tS;
    const double f = span > 0.0 ? (tS - a.tS) / span : 1.0;

    TrajectorySample out;
    out.tS = tS;
    out.sIn = lerp(a.sIn, b.sIn, f);
    out.section = a.section;
    out.stop = false;
    out.xIn = lerp(a.xIn, b.xIn, f);
    out.yIn = lerp(a.yIn, b.yIn, f);
    out.headingDeg = lerp(a.headingDeg, b.headingDeg, f);
    out.speedInPerS = lerp(a.speedInPerS, b.speedInPerS, f);
    out.accelInPerS2 = lerp(a.accelInPerS2, b.accelInPerS2, f);
    out.vxInPerS = lerp(a.vxInPerS, b.vxInPerS, f);
    out.vyInPerS = lerp(a.vyInPerS, b.vyInPerS, f);
    out.omegaDegPerS = lerp(a.omegaDegPerS, b.omegaDegPerS, f);
    out.forwardPerIn = lerp(a.forwardPerIn, b.forwardPerIn, f);
    out.strafePerIn = lerp(a.strafePerIn, b.strafePerIn, f);
    out.turnDegPerIn = lerp(a.turnDegPerIn, b.turnDegPerIn, f);
    out.forwardInPerS = lerp(a.forwardInPerS, b.forwardInPerS, f);
    out.strafeInPerS = lerp(a.strafeInPerS, b.strafeInPerS, f);
    out.forwardAccelInPerS2 = lerp(a.forwardAccelInPerS2, b.forwardAccelInPerS2, f);
    out.strafeAccelInPerS2 = lerp(a.strafeAccelInPerS2, b.strafeAccelInPerS2, f);
    out.turnAccelDegPerS2 = lerp(a.turnAccelDegPerS2, b.turnAccelDegPerS2, f);
    return out;
}

const std::vector<TrajectorySample>& Trajectory::samples() const { return samples_; }

} // namespace sapphirelib::motion
