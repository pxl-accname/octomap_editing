#include <OEServer.hpp>


using namespace interactive_markers;

namespace octomap_editing
{
  OEServer::OEServer(ros::NodeHandle nh, double resolution, std::string update_topic)
    : _server(update_topic, "test", true),
      _markerFactory(resolution),
      _menu_handler(),
      _nh(nh),
      _ocPointcloud(),
      _ocTree(resolution)
  {
    m_publishFreeSpace = true;
    m_useHeightMap = true;
    createMenu();
    _sub = _nh.subscribe("/octomap_point_cloud_centers", 1000, &octomap_editing::OEServer::getPointCloudCallback, this);
    _marker_pub = _nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
  }

  void
  OEServer::openMapfile(std::string mapFilename)
  {
    std::cout << "OcTree reading binary.." << std::endl;
    _ocTree.readBinary(mapFilename);
    std::cout << "OcTree reading binary done!" << std::endl;

    std::cout << "OcServer open file..." << std::endl;
    openFile(mapFilename);
    std::cout << "OcServer open file done!" << std::endl;
  }

  void
  OEServer::createInteractiveMarker(octomap::point3d center_coords)
  {
    // creates the interactive marker
    _imarker = _markerFactory.createControlMarker(center_coords, octomap_editing::MENU);
    _imarker_pose_initial = _imarker.pose;
    _server.insert(_imarker);
    _server.setCallback(_imarker.name, boost::bind(&octomap_editing::OEServer::processMarkerFeedback, this, _1));
    _menu_handler.apply(_server, _imarker.name);
    _server.applyChanges();
  }

  void
  OEServer::createTextMarker(octomath::Pose6D pose)
  {
    _imarker_text = _markerFactory.createTextMarker(pose);
    _server.insert(_imarker_text);
    _server.applyChanges();
  }

  void
  OEServer::createMenu()
  {
    // parent menu entries
    interactive_markers::MenuHandler::EntryHandle move_axis_entry = _menu_handler.insert("Move Axis");
    interactive_markers::MenuHandler::EntryHandle rotate_axis_entry = _menu_handler.insert("Rotate Axis");
    interactive_markers::MenuHandler::EntryHandle done_entry = _menu_handler.insert("Apply changes!", boost::bind(&octomap_editing::OEServer::applyChangesFeedback, this, _1));
    interactive_markers::MenuHandler::EntryHandle reset_entry = _menu_handler.insert("Reset changes!", boost::bind(&octomap_editing::OEServer::resetChangesFeedback, this, _1));

    // TODO: add reset menu entry

    // move part
    interactive_markers::MenuHandler::EntryHandle move_axis_x_entry = _menu_handler.insert(move_axis_entry, "X", boost::bind(&octomap_editing::OEServer::menuMoveAxisFeedback, this, _1));
    _menu_handler.setCheckState(move_axis_x_entry, MenuHandler::UNCHECKED);
    _menuentry_to_axis_map.insert(std::pair<uint, char>(move_axis_x_entry, 'x'));

    interactive_markers::MenuHandler::EntryHandle move_axis_y_entry = _menu_handler.insert(move_axis_entry, "Y", boost::bind(&octomap_editing::OEServer::menuMoveAxisFeedback, this, _1));
    _menu_handler.setCheckState(move_axis_y_entry, MenuHandler::UNCHECKED);
    _menuentry_to_axis_map.insert(std::pair<uint, char>(move_axis_y_entry, 'y'));

    interactive_markers::MenuHandler::EntryHandle move_axis_z_entry = _menu_handler.insert(move_axis_entry, "Z", boost::bind(&octomap_editing::OEServer::menuMoveAxisFeedback, this, _1));
    _menu_handler.setCheckState(move_axis_z_entry, MenuHandler::UNCHECKED);
    _menuentry_to_axis_map.insert(std::pair<uint, char>(move_axis_z_entry, 'z'));

    // rotate part
    interactive_markers::MenuHandler::EntryHandle rotate_axis_x_entry = _menu_handler.insert(rotate_axis_entry, "X", boost::bind(&octomap_editing::OEServer::menuRotateAxisFeedback, this, _1));
    _menu_handler.setCheckState(rotate_axis_x_entry, MenuHandler::UNCHECKED);
    _menuentry_to_axis_map.insert(std::pair<uint, char>(rotate_axis_x_entry, 'x'));

    interactive_markers::MenuHandler::EntryHandle rotate_axis_y_entry = _menu_handler.insert(rotate_axis_entry, "Y", boost::bind(&octomap_editing::OEServer::menuRotateAxisFeedback, this, _1));
    _menu_handler.setCheckState(rotate_axis_y_entry, MenuHandler::UNCHECKED);
    _menuentry_to_axis_map.insert(std::pair<uint, char>(rotate_axis_y_entry, 'y'));

    interactive_markers::MenuHandler::EntryHandle rotate_axis_z_entry = _menu_handler.insert(rotate_axis_entry, "Z", boost::bind(&octomap_editing::OEServer::menuRotateAxisFeedback, this, _1));
    _menu_handler.setCheckState(rotate_axis_z_entry, MenuHandler::UNCHECKED);
    _menuentry_to_axis_map.insert(std::pair<uint, char>(rotate_axis_z_entry, 'z'));
  }

  void
  OEServer::menuMoveAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    MenuHandler::CheckState check_state;
    _menu_handler.getCheckState(feedback->menu_entry_id, check_state);

    if (check_state == MenuHandler::CHECKED)
    {
      // remove the moving control
      _menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
      std::string name = "move_axis_"; // TODO: hardcoded... not so nice!
      name += _menuentry_to_axis_map.at(feedback->menu_entry_id);
      for (auto it = _imarker.controls.begin(), end = _imarker.controls.end(); it != end; ++it)
      {
        if (it->name == name)
        {
          _imarker.controls.erase(it);
        }
      }
    }
    else
    {
      // add the moving control
      _menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
      _imarker.controls.push_back(_markerFactory.createMoveAxisControl(_menuentry_to_axis_map.at(feedback->menu_entry_id)));
    }
    refreshServer(feedback->pose);
  }

  void
  OEServer::menuRotateAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    MenuHandler::CheckState check_state;
    _menu_handler.getCheckState(feedback->menu_entry_id, check_state);

    if (check_state == MenuHandler::CHECKED)
    {
      // remove the moving control!
      _menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
      std::string name = "rotate_axis_";  // TODO: hardcoded... not so nice!
      name += _menuentry_to_axis_map.at(feedback->menu_entry_id);
      for (auto it = _imarker.controls.begin(), end = _imarker.controls.end(); it != end; ++it)
      {
        if (it->name == name)
        {
          _imarker.controls.erase(it);
        }
      }
    }
    else
    {
      // add the moving controls
      _menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
      _imarker.controls.push_back(_markerFactory.createRotateAxisControl(_menuentry_to_axis_map.at(feedback->menu_entry_id)));
    }
    refreshServer(feedback->pose);
  }

  void
  OEServer::updatePointCloud()
  {
    _imarker_pose_change = calculatePoseChange();
    publishPointCloud();
  }

  void
  OEServer::refreshServer(geometry_msgs::Pose pose)
  {
    _imarker_pose = pose;
    _imarker.pose = pose;
    _server.insert(_imarker);
    _menu_handler.reApply(_server);


//    _server.erase(_imarker_text.name);
    octomath::Vector3 v(pose.position.x, pose.position.y, pose.position.z);
    octomath::Quaternion q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    octomath::Pose6D p(v,q);
    _imarker_text.pose.position = pose.position;
    _imarker_text.pose.position.z += 1; // TODO: die 1 ist noch hardcoded... als Parameter setzen?
    _server.setPose(_imarker_text.name, _imarker_text.pose);
    _server.applyChanges();
  }

  void
  OEServer::applyChangesFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    octomath::Pose6D transformMatrix_t = getTransformationMatrix(true);
    octomath::Pose6D transformMatrix_r = getTransformationMatrix(false);
    bool occupied = false;
    octomap::point3d point;
    octomap::ColorOcTree ocTree(0.05);
    //octomap::ColorOcTreeNode::Color nodeColor;

    for (auto it = _ocTree.begin_tree(), end = _ocTree.end_tree(); it != end; ++it)
    {
      // delete old node
      point = it.getCoordinate();
      occupied = (_ocTree.isNodeOccupied(*it)) ? true : false;

      point += transformMatrix_t.trans();
      point.rotate_IP(transformMatrix_r.roll(), transformMatrix_r.pitch(), transformMatrix_r.yaw());
      ocTree.updateNode(point, occupied);
      //ocTree.setNodeColor(point.x(), point.y(), point.z(), nodeColor.r, nodeColor.g, nodeColor.b);
    }

    ocTree.writeBinary("/home/accname/programming/ROS/octomap_editing/fr_transformed.bt");
    std::cout << "Transformed OcTree saved!" << std::endl;
  }

  void
  OEServer::getPointCloudCallback(const sensor_msgs::PointCloud2 &pc)
  {
    if(_ocPointcloud.size() == 0)
    {
      octomap::pointCloud2ToOctomap(pc,_ocPointcloud);
      _sub.shutdown();
      _pub = _nh.advertise<sensor_msgs::PointCloud2>("/octomap_point_cloud_centers2", 1000);
      publishPointCloud();
    }
  }

  void
  OEServer::publishPointCloud()
  {
    sensor_msgs::PointCloud2 msg;
    msg.header.frame_id = "map";
    octomap::point3d_list p3_list;
    octomath::Pose6D transformMatrix;

    transformMatrix = getTransformationMatrix(true);
    _ocPointcloud.transform(transformMatrix);
    transformMatrix = getTransformationMatrix(false);
    _ocPointcloud.rotate(transformMatrix.roll(), transformMatrix.pitch(), transformMatrix.yaw());

    for (auto it = _ocPointcloud.begin(), end = _ocPointcloud.end(); it != end; ++it)
    {
      p3_list.push_back(octomath::Vector3(it->x(), it->y(), it->z()));
    }

    sensor_msgs::PointCloud2Modifier pcd_modifier(msg);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    octomap::pointsOctomapToPointCloud2(p3_list, msg);
    _pub.publish(msg);
  }

  octomap::pose6d
  OEServer::getTransformationMatrix(bool transl)
  {
    if (transl)
    {
      octomath::Vector3 trans(static_cast<float>(_imarker_pose_change.position.x),
                               static_cast<float>(_imarker_pose_change.position.y),
                               static_cast<float>(_imarker_pose_change.position.z));
      octomath::Quaternion rotat(static_cast<float>(1),
                                 static_cast<float>(0),
                                 static_cast<float>(0),
                                 static_cast<float>(0));
      octomap::pose6d transformMatrix(trans, rotat);
      return transformMatrix;
    }
    else
    {
      octomath::Vector3 trans(static_cast<float>(0),
                               static_cast<float>(0),
                               static_cast<float>(0));
      octomath::Quaternion rotat(static_cast<float>(1),
                                 static_cast<float>(_imarker_pose_change.orientation.x),
                                 static_cast<float>(_imarker_pose_change.orientation.y),
                                 static_cast<float>(_imarker_pose_change.orientation.z));
      octomap::pose6d transformMatrix(trans, rotat);
      return transformMatrix;
    }
  }

  geometry_msgs::Pose
  OEServer::calculatePoseChange()
  {
    geometry_msgs::Pose p_change;
    p_change.orientation.w = 1;
    p_change.position.x = _imarker_pose_mu.position.x - _imarker_pose_md.position.x;
    p_change.position.y = _imarker_pose_mu.position.y - _imarker_pose_md.position.y;
    p_change.position.z = _imarker_pose_mu.position.z - _imarker_pose_md.position.z;

    p_change.orientation.x = _imarker_pose_mu.orientation.x - _imarker_pose_md.orientation.x;
    p_change.orientation.y = _imarker_pose_mu.orientation.y - _imarker_pose_md.orientation.y;
    p_change.orientation.z = _imarker_pose_mu.orientation.z - _imarker_pose_md.orientation.z;

    return p_change;
  }

  void
  OEServer::updateText(geometry_msgs::Pose pose)
  {
    _server.erase(_imarker_text.name);
    octomath::Vector3 v(pose.position.x, pose.position.y, pose.position.z);
    octomath::Quaternion q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    octomath::Pose6D p(v,q);
    _imarker_text.controls[0].markers[0].text = _markerFactory.formatText(p.x(), p.y(), p.z(), p.roll(), p.pitch(), p.yaw());
    _server.insert(_imarker_text);
    _server.applyChanges();
  }

  void
  OEServer::processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN)
    {
      _imarker_pose_md = feedback->pose;
    }
    else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    {
      _imarker_pose_mu = feedback->pose;
      updatePointCloud();
      updateText(feedback->pose);
      refreshServer(feedback->pose);
    }
  }

  void
  OEServer::resetChangesFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    // reset the imarker and the pointcloud
    // for the pointcloud --> take current pose and calc the change
    // for the marker --> set the correct pose

    //careful: this is an ugly hack!
    geometry_msgs::Point p;
    p.x = 0.;
    p.y = 0.;
    p.z = 0.;
    geometry_msgs::Quaternion q;
    q.w = 1.;
    q.x = 0.;
    q.y = 0.;
    q.z = 0.;
    geometry_msgs::Pose po;
    po.position = p;
    po.orientation = q;
    _imarker_pose = po;
    _imarker_pose_md = feedback->pose;
    _imarker_pose_mu = po;
    updatePointCloud();
    updateText(po);
    refreshServer(po);
  }

  void
  OEServer::testLineMarker()
  {

    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = 0.1;
    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;



    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 10; ++i)
    {
      float y = 1;
      float z = 1;

      geometry_msgs::Point p;
      p.x = (int32_t)i;
      p.y = y;
      p.z = z;

      // The line list needs two points for each line
      line_list.points.push_back(p);
      p.z += 1.0;
      line_list.points.push_back(p);
    }
    _marker_pub.publish(line_list);
  }
}
