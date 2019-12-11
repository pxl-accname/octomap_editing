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

  bool
  OECube::polygonstuff(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3, geometry_msgs::Point p_check)
  {
    // direction vector p1 -> p2
    geometry_msgs::Point p1p2;
    p1p2.x = p2.x - p1.x;
    p1p2.y = p2.y - p1.y;
    p1p2.z = p2.z - p1.z;

    // direction vector p1 -> p3
    geometry_msgs::Point p1p3;
    p1p3.x = p3.x - p1.x;
    p1p3.y = p3.y - p1.y;
    p1p3.z = p3.z - p1.z;

    // normal vector
    geometry_msgs::Point pn;
    pn.x = p1p2.y * p1p3.z - p1p3.y * p1p2.z;
    pn.y = p1p2.z * p1p3.x - p1p3.z * p1p2.x;
    pn.z = p1p2.x * p1p3.y- p1p3.x * p1p2.y;

    // d of the scalar plane equation
    double d = -(p1.x * pn.x + p1.y * pn.y + p1.z * pn.z);

    // centroid of the traingle
    geometry_msgs::Point centroid;
    centroid.x = (p1.x + p2.x + p3.x) / 3;
    centroid.y = (p1.y + p2.y + p3.y) / 3;
    centroid.z = (p1.z + p2.z + p3.z) / 3;

    // distance from p_check to the plane
    double distance;
    distance = (pn.x * p_check.x + pn.y * p_check.y + pn.z * p_check.z + d) / (sqrt(pow(pn.x, 2) + pow(pn.y, 2) + pow(pn.z, 2)));

    return false;
  }

  void
  OECube::createLines()
  {
    // markers 000, 110, 011, 101 are needed
    for (auto it = _markers.begin(), end = _markers.end(); it != end; ++it)
    {
      auto p = std::find(_neededMarkers.begin(), _neededMarkers.end(), (*it)->getId());
      if (p != _neededMarkers.end())
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

  visualization_msgs::Marker
  OECube::getLines(uint seq)
  {
    // create line_list here
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "map";
    line_list.header.stamp = ros::Time::now();
    line_list.header.seq = seq;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.lifetime = ros::Duration();
    line_list.ns = "";
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    // line.scale.y = 0.5;
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;
    line_list.pose.orientation.w = 1;

    for (auto it = _lines.begin(), end = _lines.end(); it != end; ++it)
    {
      line_list.points.push_back((*it)->getStartPoint());
      line_list.points.push_back((*it)->getStopPoint());
    }
    return line_list;
  }

  visualization_msgs::Marker
  OECube::getTriangles(uint seq)
  {
    // create line_list here
    visualization_msgs::Marker triangle_list;
    triangle_list.header.frame_id = "map";
    triangle_list.header.stamp = ros::Time::now();
    triangle_list.header.seq = seq;
    triangle_list.action = visualization_msgs::Marker::ADD;
    triangle_list.lifetime = ros::Duration();
    triangle_list.ns = "";
    triangle_list.type = visualization_msgs::Marker::TRIANGLE_LIST;
    triangle_list.scale.x = 1;
    triangle_list.scale.y = 1;
    triangle_list.scale.z = 1;
    triangle_list.color.b = 1.0;
    triangle_list.color.a = 1.0;
    triangle_list.pose.orientation.w = 1;

    for (auto it = _markers.begin(), end = _markers.end(); it != end; ++it)
    {
      std::vector<std::shared_ptr<OECubeMarker>> neighbours = (*it)->getNeighbours();
      uint amount_neightbours = neighbours.size();
      for (size_t i = 0; i < amount_neightbours; ++i)
      {
        triangle_list.points.push_back((*it)->getMarker()->pose.position); // the marker itself
        triangle_list.points.push_back(neighbours.at(i)->getMarker()->pose.position);
        triangle_list.points.push_back(neighbours.at((i + 1) % amount_neightbours)->getMarker()->pose.position);
      }
    }
    return triangle_list;
  }

  bool
  OECube::checkPointToCorners(octomap::point3d point, std::shared_ptr<OECubeMarker> marker)
  {
    std::string marker_name = marker->getId();
    visualization_msgs::InteractiveMarker imarker = *marker->getMarker();
    bool result = true;
    if (marker_name[0] == 0)
    {
      result = (point.x() > imarker.pose.position.x) ? true : false ;
    }
    else
    {
      result = (point.x() < imarker.pose.position.x) ? true : false ;
    }

    if (marker_name[1] == 0)
    {
      result = (point.y() > imarker.pose.position.y) ? true : false ;
    }
    else
    {
      result = (point.y() < imarker.pose.position.y) ? true : false ;
    }

    if (marker_name[2] == 0)
    {
      result = (point.z() > imarker.pose.position.z) ? true : false ;
    }
    {
      result = (point.z() < imarker.pose.position.z) ? true : false ;
    }
  }

  bool
  OECube::isPointInBox(octomap::point3d point)
  {
    bool result = true;
    // takes to long!
    for (auto it = _markers.begin(), end = _markers.end(); it != end; ++it)
    {
      if (!checkPointToCorners(point, (*it)))
      {
        result = false;
      }
    }
    return result;
  }

}
