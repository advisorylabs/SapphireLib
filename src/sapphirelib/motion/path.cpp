#include "sapphirelib/motion/path.hpp"

#include <utility>

namespace sapphirelib::motion {

Path::Path(std::vector<Waypoint> waypoints) : waypoints_(std::move(waypoints)) {}

const std::vector<Waypoint>& Path::waypoints() const { return waypoints_; }

} // namespace sapphirelib::motion
