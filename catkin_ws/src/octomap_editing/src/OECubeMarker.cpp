#include <OECubeMarker.hpp>

namespace octomap_editing
{
  OECubeMarker::OECubeMarker(std::string id, visualization_msgs::InteractiveMarker marker)
    : _id(id),
      _marker(std::make_shared<visualization_msgs::InteractiveMarker>(marker))
  {}

  void
  OECubeMarker::addNeighbour(OECubeMarker neightbour)
  {
    _neighbours.push_back(std::make_shared<OECubeMarker>(neightbour));
  }

  std::vector<std::shared_ptr<OECubeMarker>>
  OECubeMarker::getNeighbours()
  {
    return _neighbours;
  }

  std::string
  OECubeMarker::getId()
  {
    return _id;
  }

  visualization_msgs::InteractiveMarker
  OECubeMarker::getMarker()
  {
    return (*_marker);
  }

  bool
  OECubeMarker::isNeighbour(OECubeMarker neighbour)
  {
    std::string nId = neighbour.getId();
    uint counter = 0;
    for (size_t i = 0; i < 3; ++i)
    {
      if (nId[i] != _id[i])
      {
        ++counter;
      }
    }
    return (counter == 1) ? true : false;
  }
}
