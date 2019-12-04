#include <OECubeLine.hpp>

namespace octomap_editing
{
  OECubeLine::OECubeLine(std::shared_ptr<OECubeMarker> start, std::shared_ptr<OECubeMarker> stop)
    : _start(start),
      _stop(stop)
  {}

  visualization_msgs::Marker
  OECubeLine::getMarker()
  {
    visualization_msgs::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = ros::Time::now();
    line.action = visualization_msgs::Marker::ADD;
    line.ns = "";
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.scale.x = 0.1;
    line.scale.y = 0.1;
    line.color.b = 1.0;
    line.color.a = 1.0;
    line.pose.orientation.w = 1;

    line.points.push_back(_start);
    return line;
  }






  //    geometry_msgs::Point p;
  //    p.x = 1.0;
  //    p.y = 1.0;
  //    p.z = 1.0;
  //    line.points.push_back(p);
  //    p.z = 10.0;
  //    line.points.push_back(p);
  //    _marker_pub.publish(line);
}
