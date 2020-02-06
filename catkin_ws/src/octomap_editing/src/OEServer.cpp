#include <OEServer.hpp>

using namespace interactive_markers;

namespace octomap_editing
{

  OEServer::OEServer(double resolution, std::string mapFilename, std::string update_topic)
    : octomap_server::OctomapServer(),
      _mapFilename(mapFilename),
      _server(update_topic, "MarkerServer", true),
      _markerFactory(resolution),
      _resolution(resolution),
      _nh(ros::NodeHandle("octomap_editing")),
      _ocPointcloud(),
      _history_pose(),
      _pc_cm_difference(),
      _sub_pc(_nh.subscribe("/octomap_point_cloud_centers", 1, &octomap_editing::OEServer::getPointCloudCallback, this)),
      _pub_pc(_nh.advertise<sensor_msgs::PointCloud2>("/octomap_point_cloud_centers2", 1000)),
      _pub_lines(_nh.advertise<visualization_msgs::Marker>("/visualization_marker_lines", 1))
  {
    openMapfile();

    // initially needed markers
    createControlMarker("imarker_pointcloud");
    createTextMarker("imarker_text");
    createCube();
    refreshServer();
  }


  void
  OEServer::openMapfile()
  {
    if (openFile(_mapFilename))
    {
      std::cout << "OCtomap " << _mapFilename << " file was successfully loaded!" << std::endl;
    }
  }


  /*
   * This function saves the whole octomap and will optionally deletes the points in the cube.
   */
  void
  OEServer::saveMap(bool delete_points)
  {
    std::cout << "Startig to save Octomap..." << std::endl;
    octomap::ColorOcTree ocTree(_resolution);
    octomap::point3d sensorOrigin(0, 0, 0);
    ocTree.insertPointCloud(_ocPointcloud, sensorOrigin);

    if (delete_points)
    {
      std::cout << "Deleting points in the cube... " << delete_points << std::endl;
      std::vector<octomap::OcTreeKey> point_keys = _cube.checkPointInBox(std::make_shared<octomap::ColorOcTree>(ocTree));
      for (auto it = point_keys.begin(), end = point_keys.end(); it != end; ++it)
      {
        ocTree.deleteNode(*it);
      }
      std::cout << "Deleting points is done." << delete_points << std::endl;
    }

    ocTree.writeBinary("/home/accname/programming/ROS/octomap_editing/fr_078_tidyup.bt");
    _ocPointcloud.clear();
    openMapfile();
    refreshServer();
    std::cout << "... Octomap has been successfully saved!" << std::endl;
  }


  /*
   * This function takes the pointcloud published by OctomapServer and save it.
   * This pointcloud will be used to visualize the transformation of the octomap.
   */
  void
  OEServer::getPointCloudCallback(const sensor_msgs::PointCloud2 &pc)
  {
    if(_ocPointcloud.size() == 0)
    {
      octomap::pointCloud2ToOctomap(pc, _ocPointcloud);

      // fill the history vector with an initial entry
      if (_history_pose.size() == 0)
      {
        _history_pose.push_back(octomath::Pose6D(0., 0., 0., 0., 0., 0.));
      }
      publishPointCloud();
    }
  }


  /*
   * This functions is responsible to publish the pointcloud everytime the control marker is moved.
   */
  void
  OEServer::publishPointCloud()
  {
    sensor_msgs::PointCloud2 msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.header.seq = _markerFactory.getNextSeq();

    octomap::point3d_list p3_list;
    octomath::Pose6D transformMatrix;

    transformMatrix = calculateLatestPoseChange();
    _ocPointcloud.transform(transformMatrix);

    for (auto it = _ocPointcloud.begin(), end = _ocPointcloud.end(); it != end; ++it)
    {
      p3_list.push_back(octomath::Vector3(it->x(), it->y(), it->z()));
    }

    sensor_msgs::PointCloud2Modifier pcd_modifier(msg);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    octomap::pointsOctomapToPointCloud2(p3_list, msg);
    _pub_pc.publish(msg);
  }


  /*
   * This function takes the two latest poses stored in the history vector
   * and calculates the difference between them in order to transform the pointcloud.
   */
  octomath::Pose6D
  OEServer::calculateLatestPoseChange()
  {
    if (_history_pose.size() > 1)
    {
      octomath::Pose6D current = _history_pose.back();
      octomath::Pose6D last = _history_pose.at(_history_pose.size() - 2);

      octomath::Vector3 trans = current.trans() - last.trans();
      octomath::Quaternion rot = current.rot() * last.rot().inv();
      octomath::Pose6D result(trans, rot);
      return result;
    }
    else
    {
      octomath::Pose6D result = _history_pose.back();
      return result;
    }
  }


  /*
   * This function creates the menu for the control marker.
   */
  interactive_markers::MenuHandler
  OEServer::createControlMarkerMenu()
  {
    interactive_markers::MenuHandler menu_handler;

    // parent menu entries
    interactive_markers::MenuHandler::EntryHandle translation_axis_entry = menu_handler.insert("Translation axis");
    interactive_markers::MenuHandler::EntryHandle rotate_axis_entry = menu_handler.insert("Rotation axis");
    interactive_markers::MenuHandler::EntryHandle reset_entry = menu_handler.insert("Reset changes ", boost::bind(&octomap_editing::OEServer::resetChangesCallback, this, _1));

    // translation part
    interactive_markers::MenuHandler::EntryHandle translation_axis_x_entry = menu_handler.insert(translation_axis_entry, "X", boost::bind(&octomap_editing::OEServer::menuTranslationAxisCallback, this, _1));
    menu_handler.setCheckState(translation_axis_x_entry, MenuHandler::UNCHECKED);
    _mapping_menuentry_axis[translation_axis_x_entry] = "X";

    interactive_markers::MenuHandler::EntryHandle translation_axis_y_entry = menu_handler.insert(translation_axis_entry, "Y", boost::bind(&octomap_editing::OEServer::menuTranslationAxisCallback, this, _1));
    menu_handler.setCheckState(translation_axis_y_entry, MenuHandler::UNCHECKED);
    _mapping_menuentry_axis[translation_axis_y_entry] = "Y";

    interactive_markers::MenuHandler::EntryHandle translation_axis_z_entry = menu_handler.insert(translation_axis_entry, "Z", boost::bind(&octomap_editing::OEServer::menuTranslationAxisCallback, this, _1));
    menu_handler.setCheckState(translation_axis_z_entry, MenuHandler::UNCHECKED);
    _mapping_menuentry_axis[translation_axis_z_entry] = "Z";

    interactive_markers::MenuHandler::EntryHandle translation_axis_addAll_entry = menu_handler.insert(translation_axis_entry, "Add all translation axes", boost::bind(&octomap_editing::OEServer::menuAddRemoveTranslationAxisCallback, this, _1));
    _mapping_menuentry_axis[translation_axis_addAll_entry] = "addAll";

    interactive_markers::MenuHandler::EntryHandle translation_axis_removeAll_entry = menu_handler.insert(translation_axis_entry, "Remove all translation axes", boost::bind(&octomap_editing::OEServer::menuAddRemoveTranslationAxisCallback, this, _1));
    _mapping_menuentry_axis[translation_axis_removeAll_entry] = "removeAll";



    // rotate part
    interactive_markers::MenuHandler::EntryHandle rotate_axis_x_entry = menu_handler.insert(rotate_axis_entry, "X", boost::bind(&octomap_editing::OEServer::menuRotateAxisCallback, this, _1));
    menu_handler.setCheckState(rotate_axis_x_entry, MenuHandler::UNCHECKED);
    _mapping_menuentry_axis[rotate_axis_x_entry] = "X";


    interactive_markers::MenuHandler::EntryHandle rotate_axis_y_entry = menu_handler.insert(rotate_axis_entry, "Y", boost::bind(&octomap_editing::OEServer::menuRotateAxisCallback, this, _1));
    menu_handler.setCheckState(rotate_axis_y_entry, MenuHandler::UNCHECKED);
    _mapping_menuentry_axis[rotate_axis_y_entry] = "Y";


    interactive_markers::MenuHandler::EntryHandle rotate_axis_z_entry = menu_handler.insert(rotate_axis_entry, "Z", boost::bind(&octomap_editing::OEServer::menuRotateAxisCallback, this, _1));
    menu_handler.setCheckState(rotate_axis_z_entry, MenuHandler::UNCHECKED);
    _mapping_menuentry_axis[rotate_axis_z_entry] = "Z";

    interactive_markers::MenuHandler::EntryHandle rotate_axis_addAll_entry = menu_handler.insert(rotate_axis_entry, "Add all rotation axes", boost::bind(&octomap_editing::OEServer::menuAddRemoveRotateAxisCallback, this, _1));
    _mapping_menuentry_axis[rotate_axis_addAll_entry] = "addAll";

    interactive_markers::MenuHandler::EntryHandle rotate_axis_removeAll_entry = menu_handler.insert(rotate_axis_entry, "Remove all rotation axes", boost::bind(&octomap_editing::OEServer::menuAddRemoveRotateAxisCallback, this, _1));
    _mapping_menuentry_axis[rotate_axis_removeAll_entry] = "removeAll";

    return menu_handler;
  }


  octomath::Pose6D
  OEServer::getPClatestPose()
  {
    octomath::Pose6D result(0., 0., 0., 0., 0., 0.);
    if (_history_pose.size() > 0)
    {
      result = _history_pose.back();
    }
    return result;
  }


  /*
   * This function refreshes the marker, menu etc.
   * An improvement of this function would be to make it only partial refreshing.
   * Right now it doesn't care and refreshes everything.
   */
  void
  OEServer::refreshServer()
  {
    updateText();
    for (auto it = _markers.begin(), end = _markers.end(); it != end; ++it)
    {
      _server.insert(*(it->second));
    }

    std::vector<std::shared_ptr<OECubeMarker>> cube_markers = _cube.getCubeMarkers();
    for (auto it = cube_markers.begin(), end = cube_markers.end(); it != end; ++it)
    {
      std::shared_ptr<visualization_msgs::InteractiveMarker> marker = (*it)->getMarker();
      _server.insert(*marker);
    }

    // create one line list and publish that
    visualization_msgs::Marker lines = _cube.getLines(_markerFactory.getNextSeq());
    _pub_lines.publish(lines);

    for (auto it = _menus.begin(), end = _menus.end(); it != end; ++it)
    {
      (*(it->second)).reApply(_server);
    }

    _server.applyChanges();
  }


  /*
   * This function calculates the next pose for the pointcloud.
   */
  void
  OEServer::calculatePoseChange(octomath::Pose6D new_pose)
  {
    octomath::Vector3 diff(2, 2, 2);
    octomath::Vector3 result_trans = new_pose.trans() - diff;
    octomath::Quaternion result_rot = new_pose.rot();

    octomath::Pose6D result(result_trans, result_rot);
    _history_pose.push_back(result);
  }


  void
  OEServer::updateText()
  {
    _server.erase("imarker_text");
    std::shared_ptr<visualization_msgs::InteractiveMarker> imarker_text = _markers["imarker_text"];
    std::shared_ptr<visualization_msgs::InteractiveMarker> imarker_pc = _markers["imarker_pointcloud"];
    octomath::Pose6D pose = geometryPoseToPose6D(imarker_pc->pose);
    imarker_text->controls[0].markers[0].text = _markerFactory.formatText(pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw());
    imarker_text->pose = pose6DToGeometryPose(pose);
    _server.insert(*imarker_text);
  }


  // ################################################ MARKER FUNCTIONS ################################################

  /*
   * This function creates the control marker in order to move the pointcloud around.
   * Therefore the marker and menu are created, and send to the server.
   */
  void
  OEServer::createControlMarker(std::string name)
  {
    visualization_msgs::InteractiveMarker control_marker = _markerFactory.createControlMarker(name, 2, 2, 2);
    octomath::Pose6D pose_difference(2., 2., 2., 0., 0., 0.);
    _pc_cm_difference = pose_difference;
    _markers[control_marker.name] = std::make_shared<visualization_msgs::InteractiveMarker>(control_marker);

    // creates the menu
    interactive_markers::MenuHandler menu_handler = createControlMarkerMenu();
    _menus[control_marker.name] = std::make_shared<interactive_markers::MenuHandler>(menu_handler);

    // insert marker with callback into the server
    _server.insert(*_markers[control_marker.name], boost::bind(&octomap_editing::OEServer::processMarkerCallback, this, _1));

    // apply the menu handle to the control marker
    _menus[control_marker.name]->apply(_server, control_marker.name);
  }


  /*
   * This function is responsible for creating the text marker.
   * The text will appear above the control marker.
   */
  void
  OEServer::createTextMarker(std::string name)
  {
    visualization_msgs::InteractiveMarker text_marker = _markerFactory.createTextMarker(name, 2., 2., 2.);
    _markers[text_marker.name] = std::make_shared<visualization_msgs::InteractiveMarker>(text_marker);
    _server.insert(*_markers[text_marker.name]);
  }


  /*
   * This function creates the markers for the cube
   */
  void
  OEServer::createCube()
  {
    _cube = _markerFactory.createCube();
    std::vector<std::shared_ptr<OECubeMarker>> cube_markers = _cube.getCubeMarkers();
    for (auto it = cube_markers.begin(), end = cube_markers.end(); it != end; ++it)
    {
      _markers[(*it)->getMarker()->name] = (*it)->getMarker();
      _server.insert(*(*it)->getMarker(), boost::bind(&octomap_editing::OEServer::processCubeMarkerCallback, this, _1));
    }
  }


  // ################################################ CALLBACK FUNCTIONS ################################################
  void
  OEServer::menuAddRemoveTranslationAxisCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    std::shared_ptr<visualization_msgs::InteractiveMarker> marker = _markers[feedback->marker_name];
    std::shared_ptr<interactive_markers::MenuHandler> menu = _menus[feedback->marker_name];

    if (_mapping_menuentry_axis[feedback->menu_entry_id] == "addAll") // entry == 7
    {
      marker->controls.push_back(_markerFactory.createTranslationAxisControl("translation_X"));
      marker->controls.push_back(_markerFactory.createTranslationAxisControl("translation_Y"));
      marker->controls.push_back(_markerFactory.createTranslationAxisControl("translation_Z"));
      menu->setCheckState(4, MenuHandler::CHECKED);
      menu->setCheckState(5, MenuHandler::CHECKED);
      menu->setCheckState(6, MenuHandler::CHECKED);
    }
    else if (_mapping_menuentry_axis[feedback->menu_entry_id] == "removeAll")
    {
      // do not delete the controls directly because it will mess up the for loop
      // instead the iterators are saved in a vector and iterator over after the first for loop
      std::vector<std::vector<visualization_msgs::InteractiveMarkerControl>::iterator> tmp_it;
      for (auto it = marker->controls.begin(), end = marker->controls.end(); it != end; ++it)
      {
        if (it->name == "translation_X" ||
            it->name == "translation_Y" ||
            it->name == "translation_Z")
        {
          tmp_it.push_back(it);
        }
      }

      for (auto it = tmp_it.begin(), end = tmp_it.end(); it != end; ++it)
      {
        marker->controls.erase(*it);
      }
      menu->setCheckState(4, MenuHandler::UNCHECKED);
      menu->setCheckState(5, MenuHandler::UNCHECKED);
      menu->setCheckState(6, MenuHandler::UNCHECKED);
    }
    refreshServer();
  }


  void
  OEServer:: menuAddRemoveRotateAxisCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    std::shared_ptr<visualization_msgs::InteractiveMarker> marker = _markers[feedback->marker_name];
    std::shared_ptr<interactive_markers::MenuHandler> menu = _menus[feedback->marker_name];


    if (_mapping_menuentry_axis[feedback->menu_entry_id] == "addAll")
    {
      marker->controls.push_back(_markerFactory.createRotateAxisControl("rotate_X"));
      marker->controls.push_back(_markerFactory.createRotateAxisControl("rotate_Y"));
      marker->controls.push_back(_markerFactory.createRotateAxisControl("rotate_Z"));
      menu->setCheckState(9, MenuHandler::CHECKED);
      menu->setCheckState(10, MenuHandler::CHECKED);
      menu->setCheckState(11, MenuHandler::CHECKED);
    }
    else if (_mapping_menuentry_axis[feedback->menu_entry_id] == "removeAll")
    {
      // do not delete the controls directly because it will mess up the for loop
      // instead the iterators are saved in a vector and iterator over after the first for loop
      std::vector<std::vector<visualization_msgs::InteractiveMarkerControl>::iterator> tmp_it;
      for (auto it = marker->controls.begin(), end = marker->controls.end(); it != end; ++it)
      {
        if (it->name == "rotate_X" ||
            it->name == "rotate_Y" ||
            it->name == "rotate_Z")
        {
          tmp_it.push_back(it);
        }
      }

      for (auto it = tmp_it.begin(), end = tmp_it.end(); it != end; ++it)
      {
        marker->controls.erase(*it);
      }

      menu->setCheckState(9, MenuHandler::UNCHECKED);
      menu->setCheckState(10, MenuHandler::UNCHECKED);
      menu->setCheckState(11, MenuHandler::UNCHECKED);
    }
    refreshServer();
  }


  void
  OEServer::menuTranslationAxisCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    MenuHandler::CheckState check_state;
    std::shared_ptr<visualization_msgs::InteractiveMarker> marker = _markers[feedback->marker_name];
    std::shared_ptr<interactive_markers::MenuHandler> menu = _menus[marker->name];
    menu->getCheckState(feedback->menu_entry_id, check_state);

    if (check_state == MenuHandler::CHECKED)
    {
      menu->setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
      std::string name = "translation_";
      name += _mapping_menuentry_axis[feedback->menu_entry_id];
      for (auto it = marker->controls.begin(), end = marker->controls.end(); it != end; ++it)
      {
        if (it->name == name)
        {
          marker->controls.erase(it);
        }
      }
    }
    else
    {
      menu->setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
      std::string title;
      menu->getTitle(feedback->menu_entry_id, title);
      marker->controls.push_back(_markerFactory.createTranslationAxisControl("translation_" + title));
    }
    refreshServer();
  }


  void
  OEServer::menuRotateAxisCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    MenuHandler::CheckState check_state;
    std::shared_ptr<visualization_msgs::InteractiveMarker> marker = _markers[feedback->marker_name];
    std::shared_ptr<interactive_markers::MenuHandler> menu = _menus[marker->name];
    menu->getCheckState(feedback->menu_entry_id, check_state);

    if (check_state == MenuHandler::CHECKED)
    {
      menu->setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
      std::string name = "rotate_";
      name += _mapping_menuentry_axis[feedback->menu_entry_id];
      for (auto it = marker->controls.begin(), end = marker->controls.end(); it != end; ++it)
      {
        if (it->name == name)
        {
          marker->controls.erase(it);
        }
      }
    }
    else
    {
      menu->setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
      std::string title;
      menu->getTitle(feedback->menu_entry_id, title);
      marker->controls.push_back(_markerFactory.createRotateAxisControl("rotate_" + title));
    }
    refreshServer();
  }


  void
  OEServer::processCubeMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    switch (feedback->event_type)
    {
      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        {} break;

      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        {
          std::shared_ptr<visualization_msgs::InteractiveMarker> imarker = _markers[feedback->marker_name];
          std::vector<std::shared_ptr<OECubeMarker>> cube_markers = _cube.getCubeMarkers();

          for (auto it = cube_markers.begin(), end = cube_markers.end(); it != end; ++it)
          {
            if ((*it)->getId() == feedback->marker_name && (*it)->checkCoordsConstraints(feedback->pose.position))
            {
              imarker->pose = feedback->pose;
              break;
            }
          }
          refreshServer();
        }
        break;
    }
  }


  // todo: look over this
  void
  OEServer::resetChangesCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    std::shared_ptr<visualization_msgs::InteractiveMarker> imarker = _markers["imarker_pointcloud"];

    octomath::Quaternion quaternion(1., 0., 0., 0.);
    octomath::Vector3 vector3(0., 0., 0.);
    octomath::Pose6D pose (vector3, quaternion);

    calculatePoseChange(pose);
    publishPointCloud();
    refreshServer();
  }


  void
  OEServer::processMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    switch (feedback->event_type)
    {
      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        {} break;

      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        {
          if (_markers.find(feedback->marker_name) != _markers.end())
          {
            if (feedback->marker_name == "imarker_pointcloud")
            {
              octomath::Pose6D pose = geometryPoseToPose6D(feedback->pose);
              calculatePoseChange(pose);
              std::shared_ptr<visualization_msgs::InteractiveMarker> imarker = _markers[feedback->marker_name];
              imarker->pose = feedback->pose;
              publishPointCloud();
            }
            refreshServer();
          }
          else
          {
            // marker nicht gefunden
          }
        }
        break;
    }
  }

  // ################################################ CONVERSION FUNCTIONS ################################################
  octomath::Pose6D
  OEServer::geometryPoseToPose6D(geometry_msgs::Pose geometryPose)
  {
    octomath::Quaternion quaternion(
          geometryPose.orientation.w,
          geometryPose.orientation.x,
          geometryPose.orientation.y,
          geometryPose.orientation.z
    );
    octomath::Vector3 vector3(
          geometryPose.position.x,
          geometryPose.position.y,
          geometryPose.position.z
    );
    octomath::Pose6D result(vector3, quaternion);
    return result;
  }

  // TODO: TESTEN!
  geometry_msgs::Pose
  OEServer::pose6DToGeometryPose(octomath::Pose6D octoPose)
  {
    // Abbreviations for the various angular functions
    // TODO: is that correct??
    double cy = cos(octoPose.yaw() * 0.5);
    double sy = sin(octoPose.yaw() * 0.5);
    double cp = cos(octoPose.pitch() * 0.5);
    double sp = sin(octoPose.pitch() * 0.5);
    double cr = cos(octoPose.roll() * 0.5);
    double sr = sin(octoPose.roll() * 0.5);

    geometry_msgs::Quaternion quaternion;
    quaternion.w = cy * cp * cr + sy * sp * sr;
    quaternion.x = cy * cp * sr - sy * sp * cr;
    quaternion.y = sy * cp * sr + cy * sp * cr;
    quaternion.z = sy * cp * cr - cy * sp * sr;

    geometry_msgs::Point point;
    point.x = octoPose.x();
    point.y = octoPose.y();
    point.z = octoPose.z();

    geometry_msgs::Pose result;
    result.orientation = quaternion;
    result.position = point;
    return result;
  }


  // ################################################ HELPER FUNCTIONS ################################################
  std::string
  OEServer::printCoords(octomath::Pose6D pose)
  {
    std::stringstream result;
    result << "(" << pose.x() << ", " << pose.y() << ", " << pose.z() << ")";
    return result.str();
  }

}
