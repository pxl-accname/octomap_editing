#ifndef OECUBE_H
#define OECUBE_H

#include <OECubeMarker.hpp>
#include <OECubeLine.hpp>

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
    std::vector<std::shared_ptr<OECubeLine>> getLines();

  private:
    std::vector<std::shared_ptr<OECubeMarker>> _markers;
    std::vector<std::shared_ptr<OECubeLine>> _lines;
    uint _lines_counter;
  };
}

#endif // OECUBE_H
