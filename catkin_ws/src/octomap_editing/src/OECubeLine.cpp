#include <OECubeLine.hpp>

namespace octomap_editing
{
  OECubeLine::OECubeLine(std::shared_ptr<OECubeMarker> start, std::shared_ptr<OECubeMarker> stop)
    : _start(start),
      _stop(stop)
  {}

  geometry_msgs::Point
  OECubeLine::getStartPoint()
  {
    return _start->getMarker()->pose.position;
  }


  geometry_msgs::Point
  OECubeLine::getStopPoint()
  {
    return _stop->getMarker()->pose.position;
  }
}
