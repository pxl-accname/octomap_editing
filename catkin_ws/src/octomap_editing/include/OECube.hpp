#ifndef OECUBE_H
#define OECUBE_H

#include <OECubeMarker.hpp>
#include <OECubeLine.hpp>
#include <OEPlane.hpp>
#include <octomap_server/OctomapServer.h>

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
    visualization_msgs::Marker getTriangles(uint seq);
    std::vector<octomap::OcTreeKey> checkPointInBox(std::shared_ptr<octomap::ColorOcTree> sp_ocTree);
    octomap::point3d pointToPoint3d(geometry_msgs::Point p);

  private:
    bool checkPointToCorners(octomap::point3d point, std::shared_ptr<OECubeMarker> marker);

    std::vector<std::shared_ptr<OECubeMarker>> _markers;
    std::vector<std::shared_ptr<OECubeLine>> _lines;
    uint _lines_counter;
    std::vector<std::string> _neededMarkers = {"000", "110", "011", "101"};
  };
}

#endif // OECUBE_H
