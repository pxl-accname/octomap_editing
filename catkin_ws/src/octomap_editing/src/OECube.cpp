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


  std::vector<octomap::OcTreeKey>
  OECube::checkPointInBox(std::shared_ptr<octomap::ColorOcTree> sp_ocTree)
  {
    octomap::point3d A, B, C, AB, AC, n, centroid_3d(0., 0., 0.);
    std::vector<OEPlane> planes;
    int sign;
    double d, distance;
    std::vector<octomap::OcTreeKey> points_in_box;

    for (auto it = _markers.begin(), end = _markers.end(); it != end; ++it)
    {
      std::vector<std::shared_ptr<OECubeMarker>> neighbours = (*it)->getNeighbours();
      uint amount_neightbours = neighbours.size();
      sign = (*it)->getSign();
      A = pointToPoint3d((*it)->getMarker()->pose.position);
      centroid_3d += A;
      for (size_t i = 0; i < amount_neightbours; ++i)
      {
        B = pointToPoint3d(neighbours.at(i)->getMarker()->pose.position);
        C = pointToPoint3d(neighbours.at((i + 1) % amount_neightbours)->getMarker()->pose.position);

        AB = B - A;
        AC = C - A;

        n = AB.cross(AC);
        d = n.dot(A);

        OEPlane plane(n, d, sign);
        planes.push_back(plane);
      }
    }
    centroid_3d /= _markers.size();

    // the initial signs of the planes should be correct!
    for (auto it_p = planes.begin(), end_p = planes.end(); it_p != end_p; ++it_p)
    {
      distance = it_p->pointPlaneDistance(centroid_3d);
      if (distance >= 0)
      {
        it_p->setSign(1);
      }
      else
      {
        it_p->setSign(-1);
      }
    }

    bool in_box = true;
    for(auto it = sp_ocTree->begin_leafs(), end = sp_ocTree->end_leafs(); it != end; ++it)
    {
      octomap::point3d point = it.getCoordinate();
      octomap::OcTreeKey indexKey = it.getKey();
      in_box = true;

      if  (sp_ocTree->isNodeOccupied(*it))
      {
        for (auto it_p = planes.begin(), end_p = planes.end(); it_p != end_p; ++it_p)
        {
          distance = it_p->pointPlaneDistance(point);
          sign = it_p->getSign();
          if (!it_p->checkDistancePointPlane(point))
          {
            in_box = false;
            break;
          }
        }
        if (in_box)
        {
          points_in_box.push_back(indexKey);
        }
      }
    }

    return points_in_box;
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
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "map";
    line_list.header.stamp = ros::Time::now();
    line_list.header.seq = seq;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.lifetime = ros::Duration();
    line_list.ns = "";
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
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


  /*
   * This function check if a cube markers new pose is over the bounderies.
   * That means it checks if the coords of two neighbours "collide"
   */
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


  octomap::point3d
  OECube::pointToPoint3d(geometry_msgs::Point p)
  {
    octomap::point3d v(p.x, p.y, p.z);
    return v;
  }

}
