#ifndef OEMARKERFACTORY_HPP
#define OEMARKERFACTORY_HPP

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <octomap/ColorOcTree.h>
#include <OECubeMarker.hpp>
#include <OECube.hpp>

namespace octomap_editing
{
  class OEMarkerFactory
  {

  public:
    OEMarkerFactory(double resolution);

    // function to create the control marker
    visualization_msgs::InteractiveMarker createControlMarker(std::string name = "", double x = 0.0, double y = 0.0, double z = 0.0);
    visualization_msgs::InteractiveMarker createTextMarker(std::string name = "", double x = 0.0, double y = 0.0, double z = 0.0);
    OECube createCube();

    visualization_msgs::InteractiveMarkerControl createTranslationAxisControl(std::string axis);
    visualization_msgs::InteractiveMarkerControl createRotateAxisControl(std::string axis);

    std::string formatText(double x = 0, double y = 0, double z = 0, double roll = 0, double pitch = 0, double yaw = 0);
    uint getNextSeq() { return _marker_count++; }

  private:
    void setResolution(double resolution) { _resolution = resolution; }
    std::vector<std::shared_ptr<OECubeMarker>> makeCubeMarkers();
    std::shared_ptr<visualization_msgs::InteractiveMarker> createCubeMarker(std::string name, double x, double y, double z);
    visualization_msgs::InteractiveMarkerControl makeTranslationControl(double x = 1.0, double y = 0.0, double z = 0.0);
    std_msgs::Header makeMarkerHeader();
    geometry_msgs::Pose makeMarkerPose(double x = 0.0, double y = 0.0, double z = 0.0);
    visualization_msgs::Marker makeMarker(uint type = visualization_msgs::Marker::CUBE, std::string text = "");
    visualization_msgs::InteractiveMarkerControl makeControl(uint control_mode = visualization_msgs::InteractiveMarkerControl::NONE, std::string name = "");

    uint _marker_count = 0;
    double _resolution;
  };
}

#endif // OEMARKERFACTORY_HPP
