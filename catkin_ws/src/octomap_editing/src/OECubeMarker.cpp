#include <OECubeMarker.hpp>

namespace octomap_editing
{
  OECubeMarker::OECubeMarker(std::string id, std::shared_ptr<visualization_msgs::InteractiveMarker> marker, uint seq, int sign)
    : _id(id),
      _marker(marker),
      _seq(seq),
      _sign(sign)
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

  std::shared_ptr<visualization_msgs::InteractiveMarker>
  OECubeMarker::getMarker()
  {
    return _marker;
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

  bool
  OECubeMarker::checkCoords(int c1, int c2, double v1, double v2)
  {
    if (c1 < c2)
    {
      return (v1 < v2) ? true : false;
    }
    else if (c1 > c2)
    {
      return (v1 > v2) ? true : false;
    }
    else
    {
      return false;
    }
  }

  bool
  OECubeMarker::checkCoordsConstraints(geometry_msgs::Point new_own_position)
  {
    bool result = true;
    for (auto it_n = _neighbours.begin(), end_n = _neighbours.end(); it_n != end_n; ++it_n)
    {
      if (result)
      {
        std::string id_n = (*it_n)->getId();
        std::shared_ptr<visualization_msgs::InteractiveMarker> marker = (*it_n)->getMarker();

        if (_id[0] != id_n[0])
        {
          result = checkCoords(_id[0], id_n[0], new_own_position.x, marker->pose.position.x);
        }
        else if (_id[1] != id_n[1])
        {
          result = checkCoords(_id[1], id_n[1], new_own_position.y, marker->pose.position.y);
        }
        else if (_id[2] != id_n[2])
        {
          result = checkCoords(_id[2], id_n[2], new_own_position.z, marker->pose.position.z);
        }
      }
    }
    return result;
  }

  int
  OECubeMarker::getSign()
  {
    return _sign;
  }

}
