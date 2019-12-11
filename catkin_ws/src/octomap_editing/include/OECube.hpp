#ifndef OECUBE_H
#define OECUBE_H

#include <OECubeMarker.hpp>
#include <OECubeLine.hpp>
#include <octomap_ros/conversions.h>

namespace octomap_editing
{
  class OECube
  {
  public:
    OECube();
    void setCubeMarkers(std::vector<std::shared_ptr<OECubeMarker>> markers);
    std::vector<std::shared_ptr<OECubeMarker>> getCubeMarkers();
    void determineNeighbourMarkers();
    void createLines();
    void insertLines(std::shared_ptr<OECubeMarker> marker);
    visualization_msgs::Marker getLines(uint seq);
    bool isPointInBox(octomap::point3d point);
    bool polygonstuff(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3, geometry_msgs::Point p_check);
    visualization_msgs::Marker getTriangles(uint seq);

  private:
    bool checkPointToCorners(octomap::point3d point, std::shared_ptr<OECubeMarker> marker);

    std::vector<std::shared_ptr<OECubeMarker>> _markers;
    std::vector<std::shared_ptr<OECubeLine>> _lines;
    uint _lines_counter;
    std::vector<std::string> _neededMarkers = {"000", "110", "011", "101"};
  };
}

#endif // OECUBE_H
