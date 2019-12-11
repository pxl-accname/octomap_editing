#ifndef OECUBELINE_H
#define OECUBELINE_H

#include <OECubeMarker.hpp>
#include <visualization_msgs/Marker.h>

namespace octomap_editing
{
  class OECubeLine
  {
  public:
    OECubeLine(std::shared_ptr<OECubeMarker> start, std::shared_ptr<OECubeMarker> stop);
    geometry_msgs::Point getStartPoint();
    geometry_msgs::Point getStopPoint();

  private:
    std::shared_ptr<OECubeMarker> _start;
    std::shared_ptr<OECubeMarker> _stop;
  };
}

#endif // OECUBELINE_H
