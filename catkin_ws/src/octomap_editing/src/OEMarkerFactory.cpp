#include <OEMarkerFactory.hpp>

namespace octomap_editing
{
  OEMarkerFactory::OEMarkerFactory(double resolution)
    : _resolution(resolution)
  {}


  visualization_msgs::InteractiveMarker
  OEMarkerFactory::createControlMarker(std::string name, double x, double y, double z)
  {
    visualization_msgs::InteractiveMarker imarker;
    imarker.header = makeMarkerHeader();
    imarker.pose = makeMarkerPose(x, y, z);
    imarker.name = name;  // + std::to_string(imarker.header.seq);

    visualization_msgs::InteractiveMarkerControl menu = makeControl(visualization_msgs::InteractiveMarkerControl::MENU);
    menu.markers.push_back(makeMarker(visualization_msgs::Marker::CUBE, ""));
    imarker.controls.push_back(menu);
    return imarker;
  }


  std::shared_ptr<visualization_msgs::InteractiveMarker>
  OEMarkerFactory::createCubeMarker(std::string name, double x, double y, double z)
  {
    visualization_msgs::InteractiveMarker imarker;
    imarker.header = makeMarkerHeader();
    imarker.pose = makeMarkerPose(x, y, z);
    imarker.name = name;

    imarker.controls.push_back(makeControl());
    imarker.controls[0].markers.push_back(makeMarker(visualization_msgs::Marker::CUBE, ""));

    imarker.controls.push_back(makeTranslationControl(1,0,0));
    imarker.controls.push_back(makeTranslationControl(0,1,0));
    imarker.controls.push_back(makeTranslationControl(0,0,1));
    return std::make_shared<visualization_msgs::InteractiveMarker>(imarker);
  }


  visualization_msgs::InteractiveMarker
  OEMarkerFactory::createTextMarker(std::string name, double x, double y, double z)
  {
    //lets try to create a text marker
    visualization_msgs::InteractiveMarker imarker_text;
    imarker_text.header = makeMarkerHeader();
    imarker_text.pose = makeMarkerPose(x, y, z);
    imarker_text.name = name; //_" + std::to_string(imarker_text.header.seq);

    visualization_msgs::InteractiveMarkerControl control = makeControl(visualization_msgs::InteractiveMarkerControl::NONE);
    std::string text = formatText(0, 0, 0, 0, 0, 0);
    control.markers.push_back(makeMarker(visualization_msgs::Marker::TEXT_VIEW_FACING, text));
    imarker_text.controls.push_back(control);
    return imarker_text;
  }


  geometry_msgs::Pose
  OEMarkerFactory::makeMarkerPose(double x, double y, double z)
  {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = 1.0;
    return pose;
  }


  visualization_msgs::Marker
  OEMarkerFactory::makeMarker(uint type, std::string text)
  {
    visualization_msgs::Marker marker;
    marker.type = type;
    marker.scale.x = 4 * _resolution;
    marker.scale.y = 4 * _resolution;
    marker.scale.z = 4 * _resolution;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.text = text;
    return marker;
  }


  std_msgs::Header
  OEMarkerFactory::makeMarkerHeader()
  {
    std_msgs::Header header;
    header.frame_id = "map";
    header.stamp = ros::Time::now();
    header.seq = getNextSeq();
    return header;
  }


  visualization_msgs::InteractiveMarkerControl
  OEMarkerFactory::makeControl(uint control_mode, std::string name)
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode = control_mode;
    control.name = name;
    control.always_visible = true;
    return control;
  }


  visualization_msgs::InteractiveMarkerControl
  OEMarkerFactory::makeTranslationControl(double x, double y, double z)
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    control.orientation.x = x;
    control.orientation.y = y;
    control.orientation.z = z;
    control.orientation.w = 1;
    control.always_visible = true;
    return control;
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


  visualization_msgs::InteractiveMarkerControl
  OEMarkerFactory::createTranslationAxisControl(std::string axis)
  {
    double x = 0.0, y = 0.0, z = 0.0;

    if (axis == "translation_X")
      x = 1.0;

    if (axis == "translation_Y")
      z = 1.0;

    if (axis == "translation_Z")
      y = 1.0;

    visualization_msgs::InteractiveMarkerControl interactive_control;
    interactive_control.name = axis;
    interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactive_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    interactive_control.orientation.x = x;
    interactive_control.orientation.y = y;
    interactive_control.orientation.z = z;
    interactive_control.orientation.w = 1;
    interactive_control.always_visible = true;
    return interactive_control;
  }


  std::vector<std::shared_ptr<OECubeMarker>>
  OEMarkerFactory::makeCubeMarkers()
  {
    std::vector<std::shared_ptr<OECubeMarker>> markers;
    markers.push_back(std::make_shared<OECubeMarker>("000", createCubeMarker("000", 0, 0, 0), getNextSeq(), 1));
    markers.push_back(std::make_shared<OECubeMarker>("100", createCubeMarker("100", 1, 0, 0), getNextSeq(), -1));
    markers.push_back(std::make_shared<OECubeMarker>("010", createCubeMarker("010", 0, 1, 0), getNextSeq(), 1));
    markers.push_back(std::make_shared<OECubeMarker>("110", createCubeMarker("110", 1, 1, 0), getNextSeq(), -1));

    markers.push_back(std::make_shared<OECubeMarker>("001", createCubeMarker("001", 0, 0, 1), getNextSeq(), -1));
    markers.push_back(std::make_shared<OECubeMarker>("101", createCubeMarker("101", 1, 0, 1), getNextSeq(), 1));
    markers.push_back(std::make_shared<OECubeMarker>("011", createCubeMarker("011", 0, 1, 1), getNextSeq(), -1));
    markers.push_back(std::make_shared<OECubeMarker>("111", createCubeMarker("111", 1, 1, 1), getNextSeq(), 1));

    return markers;
  }


  OECube
  OEMarkerFactory::createCube()
  {
    OECube cube;
    cube.setCubeMarkers(makeCubeMarkers());
    cube.determineNeighbourMarkers();
    cube.createLines();
    return cube;
  }


  visualization_msgs::InteractiveMarkerControl
  OEMarkerFactory::createRotateAxisControl(std::string axis)
  {
    double x = 0.0, y = 0.0, z = 0.0;

    if (axis == "rotate_X")
      x = 1.0;

    if (axis == "rotate_Y")
      z = 1.0;

    if (axis == "rotate_Z")
      y = 1.0;

    visualization_msgs::InteractiveMarkerControl interactive_control;
    interactive_control.name = axis;
    interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    interactive_control.orientation.x = x;
    interactive_control.orientation.y = y;
    interactive_control.orientation.z = z;
    interactive_control.orientation.w = 1;
    interactive_control.always_visible = true;
    return interactive_control;
  }

}
