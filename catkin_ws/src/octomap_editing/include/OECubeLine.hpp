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
    visualization_msgs::Marker getMarker();

  private:
    std::shared_ptr<OECubeMarker> _start;
    std::shared_ptr<OECubeMarker> _stop;
  };
}

#endif // OECUBELINE_H
