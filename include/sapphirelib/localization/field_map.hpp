#pragma once

#include <vector>

namespace sapphirelib::localization {

/**
 * @brief Wall to wall size of a standard VRC field, in inches
 *
 * The game manual allows 140.5 +-0.5in. Measure your practice field: every inch it's off shows up
 * as a bias in the localizer's estimate
 */
constexpr double kVrcFieldSizeIn = 140.5;

/**
 * @brief A straight wall or edge, in field coordinates
 */
struct Segment {
    /** one end's x, in inches */
    double x1In = 0.0;
    /** one end's y, in inches */
    double y1In = 0.0;
    /** the other end's x, in inches */
    double x2In = 0.0;
    /** the other end's y, in inches */
    double y2In = 0.0;
};

/**
 * @brief What a distance sensor can see on the field: the perimeter walls, plus anything else
 * solid you add
 *
 * Uses the same frame as odom::Pose: x to the right, y downfield, headings clockwise from +y. The
 * localizer only makes sense once odometry's pose is in this frame, so start autonomous with an
 * Odometry::setPose() in field coordinates
 *
 * Only add things that don't move and that a sensor will see at its mounting height. A game
 * element that gets pushed around does more harm in the map than out of it
 *
 * @b Example
 * @code {.cpp}
 * // a standard field with the origin in the middle, walls at +-70.25in
 * auto map = sapphirelib::localization::FieldMap::centered();
 * // and a 10x10in post in the middle of it
 * map.addBox(-5, -5, 5, 5);
 * @endcode
 */
class FieldMap {
public:
    /**
     * @brief Construct a map walled in on four sides
     *
     * @param minXIn the left wall's x, in inches
     * @param minYIn the near wall's y, in inches
     * @param maxXIn the right wall's x, in inches
     * @param maxYIn the far wall's y, in inches
     */
    FieldMap(double minXIn, double minYIn, double maxXIn, double maxYIn);

    /**
     * @brief Construct a square field with the origin in the middle
     *
     * @param sizeIn wall to wall size, in inches. kVrcFieldSizeIn by default
     * @return FieldMap walls at +-sizeIn / 2 on both axes
     */
    static FieldMap centered(double sizeIn = kVrcFieldSizeIn);

    /**
     * @brief Add a wall or edge
     *
     * @param segment the segment, in field coordinates
     */
    void addSegment(Segment segment);

    /**
     * @brief Add a solid axis-aligned box, as its four edges
     *
     * @param minXIn left edge x, in inches
     * @param minYIn near edge y, in inches
     * @param maxXIn right edge x, in inches
     * @param maxYIn far edge y, in inches
     */
    void addBox(double minXIn, double minYIn, double maxXIn, double maxYIn);

    /**
     * @brief Get the distance along a ray to the first thing it hits
     *
     * @param xIn ray start x, in inches
     * @param yIn ray start y, in inches
     * @param dirX ray direction x. (dirX, dirY) must be a unit vector
     * @param dirY ray direction y
     * @return double distance to the nearest hit, in inches. Infinity if the ray hits nothing
     */
    double castRayIn(double xIn, double yIn, double dirX, double dirY) const;

    /**
     * @brief Get the distance along a heading to the first thing it hits
     *
     * @param xIn ray start x, in inches
     * @param yIn ray start y, in inches
     * @param headingDeg ray heading, in degrees clockwise from +y, like odom::Pose
     * @return double distance to the nearest hit, in inches. Infinity if the ray hits nothing
     */
    double castRayAtHeadingIn(double xIn, double yIn, double headingDeg) const;

    /**
     * @brief Whether a point is inside the perimeter walls
     */
    bool contains(double xIn, double yIn) const;

    /**
     * @brief Get every segment, the four walls first
     */
    const std::vector<Segment>& segments() const;

    /** @brief Get the left wall's x, in inches */
    double minXIn() const;
    /** @brief Get the near wall's y, in inches */
    double minYIn() const;
    /** @brief Get the right wall's x, in inches */
    double maxXIn() const;
    /** @brief Get the far wall's y, in inches */
    double maxYIn() const;

private:
    double minXIn_;
    double minYIn_;
    double maxXIn_;
    double maxYIn_;
    std::vector<Segment> segments_;
};

} // namespace sapphirelib::localization
