#pragma once

#include <cstddef>
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
     * @brief Get the distance to the nearest thing added to the map (addSegment(), addBox()) anywhere
     * in a cone, the way a distance sensor's beam sees it
     *
     * A V5 Distance Sensor's beam is a cone, and it reads the nearest thing anywhere in it: a narrow
     * object, like a loader, well off the beam's center, which a single ray would miss. Only
     * segments turned within 60 degrees of facing the beam count: one running along the beam, like
     * the side of a box seen from beside it, sends too little back to be what the sensor reads. The
     * walls aren't counted here (castRayIn() has them, and castBeamIn() takes both)
     *
     * @param xIn the cone's tip x, in inches
     * @param yIn the cone's tip y, in inches
     * @param dirX the cone's center direction x. (dirX, dirY) must be a unit vector
     * @param dirY the cone's center direction y
     * @param halfAngleDeg how far either side of its center the cone reaches, in degrees. 0 or
     * less sees nothing
     * @return double straight-line distance from the tip to the nearest point of an added segment
     * inside the cone, in inches. Infinity if none of them is
     */
    double castConeIn(double xIn, double yIn, double dirX, double dirY, double halfAngleDeg) const;

    /**
     * @brief Get what a distance sensor's beam reads: the walls along its center ray, or anything
     * added to the map anywhere in its cone, whichever is nearer
     *
     * The walls stay the center ray's: at a slant, how short of it a wall reads is the sensor's
     * own (see MountCalibrationResult::beamShape)
     *
     * @param xIn the sensor face's x, in inches
     * @param yIn the sensor face's y, in inches
     * @param dirX the beam's direction x. (dirX, dirY) must be a unit vector
     * @param dirY the beam's direction y
     * @param halfAngleDeg the cone's half angle, in degrees (BeamModel::coneHalfAngleDeg). 0 is
     * castRayIn()
     * @return double distance to what it reads, in inches. Infinity if it would see nothing
     */
    double castBeamIn(double xIn, double yIn, double dirX, double dirY, double halfAngleDeg) const;

    /**
     * @brief Mark a stretch of the perimeter that a distance sensor can't be trusted on
     *
     * For a part of the wall that doesn't read where the wall is, which the localizer would
     * otherwise take for the wall a few inches from where it is: Override's Toggles, on top of
     * the middle of every wall, read the wall 3 to 4in long (96671H's SL000040 and SL000070), a
     * believable wall in the wrong place rather than an outlier. MonteCarloLocalizer leaves out
     * any reading whose beam lands on one (landsOnBlindStretch()). Not a segment: nothing is cast
     * against it
     *
     * @param stretch the stretch of wall, in field coordinates
     */
    void addBlindStretch(Segment stretch);

    /**
     * @brief Whether a beam lands on a blind stretch (addBlindStretch()): its center ray reaches
     * the walls within the cone's half-width at that range of one, so some of its cone is on it
     *
     * @param xIn the sensor face's x, in inches
     * @param yIn the sensor face's y, in inches
     * @param dirX the beam's direction x. (dirX, dirY) must be a unit vector
     * @param dirY the beam's direction y
     * @param halfAngleDeg the cone's half angle, in degrees (BeamModel::coneHalfAngleDeg)
     * @return true if any blind stretch is in the beam's cone where it meets the walls; false
     * with none added
     */
    bool landsOnBlindStretch(double xIn, double yIn, double dirX, double dirY,
                             double halfAngleDeg) const;

    /**
     * @brief Which wall a ray meets first, not counting anything added to the map
     *
     * @param xIn ray start x, in inches
     * @param yIn ray start y, in inches
     * @param dirX ray direction x. (dirX, dirY) must be a unit vector
     * @param dirY ray direction y
     * @param distanceIn if not null, set to the distance to that wall, in inches (infinity if none)
     * @return int the wall's index in segments(): 0 the near wall, 1 the right one, 2 the far one,
     * 3 the left one. -1 if the ray meets none
     */
    int wallHit(double xIn, double yIn, double dirX, double dirY, double* distanceIn = nullptr) const;

    /** @brief Get every blind stretch (addBlindStretch()) */
    const std::vector<Segment>& blindStretches() const;

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
    std::vector<Segment> blindStretches_;
};

/**
 * @brief FieldMap::castRayIn() for many rays pointing the same way, from different starts
 *
 * The particle filter casts one ray per particle per sensor, and every particle shares the
 * robot's heading, so all of one sensor's rays are parallel. aim() works out each segment's share
 * of the intersection once, so each castIn() is a few multiplies per segment and no division. A
 * ray starting inside the walls leaves through whichever of the two walls ahead of it is nearer,
 * so the perimeter takes two multiplies, and only the segments added to the map are checked one
 * by one. Agrees with castRayIn() up to rounding
 *
 * @b Example
 * @code {.cpp}
 * using namespace sapphirelib::localization;
 * const FieldMap map = FieldMap::centered();
 * ParallelRayCaster caster;
 * caster.aim(map, 0.0, 1.0);                // every ray points along +y
 * double wall = caster.castIn(10.0, -20.0); // as map.castRayIn(10, -20, 0, 1)
 * @endcode
 */
class ParallelRayCaster {
public:
    /**
     * @brief Point every ray one way, against a map's segments
     *
     * @note keeps its storage, so it only allocates the first time it sees a map this size
     *
     * @param map what the rays can hit. Only read here; later changes to it aren't seen
     * @param dirX ray direction x. (dirX, dirY) must be a unit vector
     * @param dirY ray direction y
     */
    void aim(const FieldMap& map, double dirX, double dirY);

    /**
     * @brief Get the distance from a start, along the aimed direction, to the first thing hit
     *
     * @param xIn ray start x, in inches
     * @param yIn ray start y, in inches
     * @return double distance to the nearest hit, in inches. Infinity if the ray hits nothing
     */
    double castIn(double xIn, double yIn) const;

private:
    // one segment, against the aimed direction: its direction (ex, ey), 1 / the cross product of
    // the ray's direction with it, and the parts of the two numerators that don't depend on the
    // ray's start
    struct Edge {
        double ex;
        double ey;
        double inverseDenominator;
        double crossT;
        double crossU;
    };

    // the walls, from the map
    double minXIn_ = 0.0;
    double minYIn_ = 0.0;
    double maxXIn_ = 0.0;
    double maxYIn_ = 0.0;
    // which wall each axis heads for (maxXIn_ or minXIn_), and 1 / the direction along it. 0 when
    // the rays run parallel to those walls, and never reach them
    double wallXIn_ = 0.0;
    double wallYIn_ = 0.0;
    double inverseDirX_ = 0.0;
    double inverseDirY_ = 0.0;

    // every segment that isn't parallel to the rays: the perimeter's first, then the rest
    std::vector<Edge> edges_;
    std::size_t perimeterEdges_ = 0;
    double dirX_ = 0.0;
    double dirY_ = 0.0;
};

/**
 * @brief FieldMap::castConeIn() for many cones pointing the same way, from different tips
 *
 * The beam's cone, as ParallelRayCaster is its center ray: every particle shares the robot's
 * heading, so all of one sensor's cones point the same way. aim() puts each added segment in the
 * cones' own frame (along the beam, and across it) once, leaving out any that don't face the beam,
 * so each castIn() is a few subtractions per segment, and a segment well clear of the cone is
 * passed over after a few more. Only the segments added to the map: the walls are the center
 * ray's. Agrees with castConeIn() up to rounding
 *
 * @b Example
 * @code {.cpp}
 * using namespace sapphirelib::localization;
 * FieldMap map = FieldMap::centered();
 * map.addBox(-70.25, -60.82, -66.51, -56.8); // a loader
 * ConeCaster cone;
 * cone.aim(map, -1.0, 0.0, 15.0);            // every cone points along -x, 15 degrees either side
 * double loader = cone.castIn(-57.6, -54.8); // as map.castConeIn(-57.6, -54.8, -1, 0, 15)
 * @endcode
 */
class ConeCaster {
public:
    /**
     * @brief Point every cone one way, against a map's added segments
     *
     * @note keeps its storage, so it only allocates the first time it sees a map this size
     *
     * @param map what the cones can see. Only read here; later changes to it aren't seen
     * @param dirX the cones' center direction x. (dirX, dirY) must be a unit vector
     * @param dirY the cones' center direction y
     * @param halfAngleDeg how far either side of its center each cone reaches, in degrees. 0 or
     * less sees nothing
     */
    void aim(const FieldMap& map, double dirX, double dirY, double halfAngleDeg);

    /**
     * @brief Get the distance from a cone's tip to the nearest added segment inside it
     *
     * @param xIn the tip's x, in inches
     * @param yIn the tip's y, in inches
     * @return double straight-line distance to the nearest point inside the cone, in inches.
     * Infinity if there's none
     */
    double castIn(double xIn, double yIn) const;

private:
    // one added segment in the cones' frame, measured from the field's origin: its ends and its
    // middle along the beam and across it, half its length, and 1 / its length squared
    struct Edge {
        double along1;
        double across1;
        double along2;
        double across2;
        double alongMiddle;
        double acrossMiddle;
        double halfLength;
        double inverseLengthSquared;
    };

    double dirX_ = 0.0;
    double dirY_ = 0.0;
    // tan(the half angle): inside the cone, |across| <= this * along
    double slope_ = 0.0;
    // sqrt(1 + slope_^2), which puts a distance in the units of that test
    double slopeNorm_ = 1.0;
    std::vector<Edge> edges_;
};

} // namespace sapphirelib::localization
