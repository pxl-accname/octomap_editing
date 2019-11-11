#include <OEMarkerFactory.hpp>

namespace octomap_editing
{
  OEMarkerFactory::OEMarkerFactory(double resolution)
    : _resolution(resolution)
  {}

  visualization_msgs::Marker
  OEMarkerFactory::makeTextMarker(std::string text)
  {
    visualization_msgs::Marker text_marker;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    text_marker.scale.z = 0.15;

    text_marker.color.r = 1;
    text_marker.color.g = 0;
    text_marker.color.b = 0;
    text_marker.color.a = 1.0;

    text_marker.text = text;

    return text_marker;
  }

  visualization_msgs::Marker
  OEMarkerFactory::makeBoxMarker()
  {
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;

    box_marker.scale.x = 0.5 * static_cast<double>(_resolution);
    box_marker.scale.y = 0.5 *static_cast<double>(_resolution);
    box_marker.scale.z = 0.5 *static_cast<double>(_resolution);

    box_marker.color.r = 1;
    box_marker.color.g = 0;
    box_marker.color.b = 0;
    box_marker.color.a = 1.0;

    return box_marker;
  }

  visualization_msgs::Marker
  OEMarkerFactory::makeHelpPlane()
  {
    visualization_msgs::Marker help_plane;
    help_plane.type = visualization_msgs::Marker::CUBE;

    help_plane.scale.x = 2;
    help_plane.scale.y = 2;
    help_plane.scale.z = 0.05;

    help_plane.color.r = 1;
    help_plane.color.g = 0;
    help_plane.color.b = 0;
    help_plane.color.a = 1.0;

    return help_plane;
  }

  visualization_msgs::InteractiveMarkerControl
  OEMarkerFactory::makeBoxControl(std::string text)
  {
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(makeTextMarker(text));
    return box_control;
  }

  void
  OEMarkerFactory::createMarkerHeader(visualization_msgs::InteractiveMarker &imarker)
  {
    imarker.header.frame_id = "map";  // TODO: make this an argument
    imarker.header.stamp = ros::Time::now();
    imarker.header.seq = _i_marker_counter;
    imarker.name = "Control marker" + std::to_string(_i_marker_counter);
    ++_i_marker_counter;
    //imarker.description = "Simple Interactive Marker with a context menu";  // TODO: add more information based on the used control!
    // the description float over the marker in rviz -> ugly...
    imarker.scale = static_cast<float>(_resolution);
  }

  void
  OEMarkerFactory::createMarkerPose(visualization_msgs::InteractiveMarker &imarker, double x, double y, double z)
  {
    imarker.pose.position.x = x;
    imarker.pose.position.y = y;
    imarker.pose.position.z = z;
  }

  visualization_msgs::InteractiveMarker
  OEMarkerFactory::createControlMarker(octomap::point3d center_coords, octomap_editing::Control_Mode control_mode)
  {
    visualization_msgs::InteractiveMarker imarker;
    // same for all markers
    createMarkerHeader(imarker);
    createMarkerPose(imarker);

    // create the desired interactive marker
    makeInteractiveControl(imarker, control_mode);
    return imarker;
  }

  std::string
  OEMarkerFactory::formatText(double x , double y, double z, double roll, double pitch, double yaw)
  {
    std::stringstream text;
    text << "x: " << x << std::endl;
    text << "y: " << y << std::endl;
    text << "z: " << z << std::endl << std::endl;

    text << "roll: " << roll << std::endl;
    text << "pitch: " << pitch << std::endl;
    text << "yaw: " << yaw;
    return text.str();
  }

  visualization_msgs::InteractiveMarker
  OEMarkerFactory::createTextMarker(octomap::pose6d pose)
  {
    //lets try to create a text marker
    visualization_msgs::InteractiveMarker imarker_text;
    createMarkerHeader(imarker_text);
    createMarkerPose(imarker_text, 0, 0, 1);
    std::string text = formatText(pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw());
    imarker_text.controls.push_back(makeBoxControl(text));
    return imarker_text;
  }

  void
  OEMarkerFactory::makeInteractiveControl(visualization_msgs::InteractiveMarker &imarker, octomap_editing::Control_Mode control_mode)
  {
    switch(control_mode)
    {
//      case octomap_editing::MOVE_X_AXIS:
//        createMoveAxisControl();
//        break;

      case octomap_editing::MENU:
        createMenu(imarker); // TODO: maybe remove the controls again
        break;
    }
  }

  visualization_msgs::InteractiveMarkerControl
  OEMarkerFactory::createMoveAxisControl(char axis)
  {
    double x = 0.0, y = 0.0, z = 0.0;
    std::string name = "move_axis_";
    name += axis;

    switch (axis)
    {
      case 'x':
        x = 1.0;
        break;

      case 'y':
        z = 1.0;
        break;

      case 'z':
        y = 1.0;
        break;
    }

    visualization_msgs::InteractiveMarkerControl interactive_control;
    interactive_control.name = name;
    interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactive_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    interactive_control.orientation.x = x;
    interactive_control.orientation.y = y;
    interactive_control.orientation.z = z;
    interactive_control.orientation.w = 1;
    interactive_control.always_visible = true;
    return interactive_control;
  }

  visualization_msgs::InteractiveMarkerControl
  OEMarkerFactory::createRotateAxisControl(char axis)
  {
    double x = 0.0, y = 0.0, z = 0.0;
    std::string name = "rotate_axis_";
    name += axis;

    switch (axis)
    {
      case 'x':
        x = 1.0;
        break;

      case 'y':
        y = 1.0;
        break;

      case 'z':
        z = 1.0;
        break;
    }

    visualization_msgs::InteractiveMarkerControl interactive_control;
    interactive_control.name = name;
    interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    interactive_control.orientation.x = x;
    interactive_control.orientation.y = y;
    interactive_control.orientation.z = z;
    interactive_control.orientation.w = 1;
    interactive_control.always_visible = true;
    return interactive_control;
  }

  void
  OEMarkerFactory::createMenu(visualization_msgs::InteractiveMarker &imarker)
  {
   visualization_msgs::InteractiveMarkerControl menu_control;
   menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
   menu_control.name = "context_menu";

   menu_control.markers.push_back(makeBoxMarker());
   // menu_control.markers.push_back(makeHelpPlane());
   menu_control.always_visible = true;

   imarker.controls.push_back(menu_control);
  }
}
