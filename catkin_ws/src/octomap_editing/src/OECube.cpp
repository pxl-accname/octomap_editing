#include <OECube.hpp>

namespace octomap_editing
{
  OECube::OECube()
    : _lines_counter(0)
  {}

  void
  OECube::setCubeMarkers(std::vector<std::shared_ptr<OECubeMarker>> markers)
  {
    _markers = markers;
  }

  std::vector<std::shared_ptr<OECubeMarker>>
  OECube::getCubeMarkers()
  {
    return _markers;
  }

  void
  OECube::determineNeighbourMarkers()
  {
    for (auto it_m = _markers.begin(), end_m = _markers.end(); it_m != end_m; ++it_m)
    {
      for (auto it_n = _markers.begin(), end_n = _markers.end(); it_n != end_n; ++it_n)
      {
        if ((*it_m)->isNeighbour(*(*it_n)))
        {
          (*it_m)->addNeighbour(*(*it_n));
        }
      }
    }
  }

  void
  OECube::createLines()
  {
    // markers 000, 110, 011, 101 are needed
    std::vector<std::string> neededMarkers = {"000", "110", "011", "101"};
    for (auto it = _markers.begin(), end = _markers.end(); it != end; ++it)
    {
      auto p = std::find(neededMarkers.begin(), neededMarkers.end(), (*it)->getId());
      if (p != neededMarkers.end())
      {
        // marker's id is in needmarkers -> create lines from it to neighbours
        insertLines(*it);
      }
    }
  }

  void
  OECube::insertLines(std::shared_ptr<OECubeMarker> marker)
  {
    // create one OECubeLine for each neighbour
    std::vector<std::shared_ptr<OECubeMarker>> neighbours = marker->getNeighbours();
    for (auto it = neighbours.begin(), end = neighbours.end(); it != end; ++it)
    {
      OECubeLine line(marker, *it);
      _lines.push_back(std::make_shared<OECubeLine>(line));
    }
  }

  std::vector<std::shared_ptr<OECubeLine>>
  OECube::getLines()
  {
    return _lines;
  }

}
