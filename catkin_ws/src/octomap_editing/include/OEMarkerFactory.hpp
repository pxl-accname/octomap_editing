#ifndef OEMARKERFACTORY_HPP
#define OEMARKERFACTORY_HPP

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <octomap/ColorOcTree.h>
#include <OEMarker.hpp>

namespace octomap_editing
{
  enum Control_Mode { MOVE_X_AXIS = 0, MENU = 1 };

  // TODO: make this factory a singleton!
  class OEMarkerFactory
  {
  public:
    OEMarkerFactory(double resolution);

    visualization_msgs::Marker makeBoxMarker();
    visualization_msgs::Marker makeTextMarker(std::string text);
    visualization_msgs::Marker makeHelpPlane();
    visualization_msgs::InteractiveMarkerControl makeBoxControl(std::string text);
    // visualization_msgs::InteractiveMarkerControl makeBoxControl();
    void createMarkerHeader(visualization_msgs::InteractiveMarker &imarker);
    void createMarkerPose(visualization_msgs::InteractiveMarker &imarker, double x = 0, double y = 0, double z = 0);

    visualization_msgs::InteractiveMarker createControlMarker(octomap::point3d center_coords, octomap_editing::Control_Mode control_mode);
    visualization_msgs::InteractiveMarker createTextMarker(octomath::Pose6D pose);
    void makeInteractiveControl(visualization_msgs::InteractiveMarker &imarker, octomap_editing::Control_Mode control_mode = MOVE_X_AXIS);

    // formatting text for text marker
    std::string formatText(double x = 0, double y = 0, double z = 0, double roll = 0, double pitch = 0, double yaw = 0);

    // different possible control types
    visualization_msgs::InteractiveMarkerControl createMoveAxisControl(char axis);
    visualization_msgs::InteractiveMarkerControl createRotateAxisControl(char axis);
    void createMenu(visualization_msgs::InteractiveMarker &imarker);

    // methods called from server
    void setResolution(double resolution) { _resolution = resolution; }

  private:
    uint _i_marker_counter = 0;
    double _resolution;
  };
}

#endif // OEMARKERFACTORY_HPP
