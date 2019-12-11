#include <OEServer.hpp>

using namespace interactive_markers;

namespace octomap_editing
{

  OEServer::OEServer(double resolution, std::string mapFilename, std::string update_topic)
    : _ocTree(resolution),
      _mapFilename(mapFilename),
      _server(update_topic, "MarkerServer", true),
      _markerFactory(resolution),
      _nh(ros::NodeHandle("octomap_editing")),
      _ocPointcloud()
  {
    m_publishFreeSpace = true;

    _sub = _nh.subscribe("/octomap_point_cloud_centers", 1000, &octomap_editing::OEServer::getPointCloudCallback, this);
    _marker_pub = _nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

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
    std::cout << "Opening map file..." << std::endl;
    _ocTree.readBinary(_mapFilename);
    openFile(_mapFilename);
    std::cout << "Map file opened!" << std::endl;
  }

  void
  OEServer::getPointCloudCallback(const sensor_msgs::PointCloud2 &pc)
  {
    if(_ocPointcloud.size() == 0)
    {
      octomap::pointCloud2ToOctomap(pc, _ocPointcloud);
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
    msg.header.stamp = ros::Time::now();
    msg.header.seq = _markerFactory.getNextSeq();

    octomap::point3d_list p3_list;
    octomath::Pose6D transformMatrix;

    // TODO: this is ugly...
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

  interactive_markers::MenuHandler
  OEServer::createMenu()
  {
    interactive_markers::MenuHandler menu_handler;

    // parent menu entries
    interactive_markers::MenuHandler::EntryHandle move_axis_entry = menu_handler.insert("Move Axis");
    interactive_markers::MenuHandler::EntryHandle rotate_axis_entry = menu_handler.insert("Rotate Axis");
    interactive_markers::MenuHandler::EntryHandle done_entry = menu_handler.insert("Apply changes!", boost::bind(&octomap_editing::OEServer::applyChangesFeedback, this, _1));
    interactive_markers::MenuHandler::EntryHandle reset_entry = menu_handler.insert("Reset changes!", boost::bind(&octomap_editing::OEServer::resetChangesFeedback, this, _1));

    // move part
    interactive_markers::MenuHandler::EntryHandle move_axis_x_entry = menu_handler.insert(move_axis_entry, "X", boost::bind(&octomap_editing::OEServer::menuMoveAxisFeedback, this, _1));
    menu_handler.setCheckState(move_axis_x_entry, MenuHandler::UNCHECKED);
    _mapping_menuentry_axis[move_axis_x_entry] = "X";

    interactive_markers::MenuHandler::EntryHandle move_axis_y_entry = menu_handler.insert(move_axis_entry, "Y", boost::bind(&octomap_editing::OEServer::menuMoveAxisFeedback, this, _1));
    menu_handler.setCheckState(move_axis_y_entry, MenuHandler::UNCHECKED);
    _mapping_menuentry_axis[move_axis_y_entry] = "Y";

    interactive_markers::MenuHandler::EntryHandle move_axis_z_entry = menu_handler.insert(move_axis_entry, "Z", boost::bind(&octomap_editing::OEServer::menuMoveAxisFeedback, this, _1));
    menu_handler.setCheckState(move_axis_z_entry, MenuHandler::UNCHECKED);
    _mapping_menuentry_axis[move_axis_z_entry] = "Z";

    interactive_markers::MenuHandler::EntryHandle move_axis_addAll_entry = menu_handler.insert(move_axis_entry, "Add all axes", boost::bind(&octomap_editing::OEServer::menuAddRemoveMoveAxisFeedback, this, _1));
    _mapping_menuentry_axis[move_axis_addAll_entry] = "addAll";

    interactive_markers::MenuHandler::EntryHandle move_axis_removeAll_entry = menu_handler.insert(move_axis_entry, "Remove all axes", boost::bind(&octomap_editing::OEServer::menuAddRemoveMoveAxisFeedback, this, _1));
    _mapping_menuentry_axis[move_axis_removeAll_entry] = "removeAll";



    // rotate part
    interactive_markers::MenuHandler::EntryHandle rotate_axis_x_entry = menu_handler.insert(rotate_axis_entry, "X", boost::bind(&octomap_editing::OEServer::menuRotateAxisFeedback, this, _1));
    menu_handler.setCheckState(rotate_axis_x_entry, MenuHandler::UNCHECKED);
    _mapping_menuentry_axis[rotate_axis_x_entry] = "X";


    interactive_markers::MenuHandler::EntryHandle rotate_axis_y_entry = menu_handler.insert(rotate_axis_entry, "Y", boost::bind(&octomap_editing::OEServer::menuRotateAxisFeedback, this, _1));
    menu_handler.setCheckState(rotate_axis_y_entry, MenuHandler::UNCHECKED);
    _mapping_menuentry_axis[rotate_axis_y_entry] = "Y";


    interactive_markers::MenuHandler::EntryHandle rotate_axis_z_entry = menu_handler.insert(rotate_axis_entry, "Z", boost::bind(&octomap_editing::OEServer::menuRotateAxisFeedback, this, _1));
    menu_handler.setCheckState(rotate_axis_z_entry, MenuHandler::UNCHECKED);
    _mapping_menuentry_axis[rotate_axis_z_entry] = "Z";

    interactive_markers::MenuHandler::EntryHandle rotate_axis_addAll_entry = menu_handler.insert(rotate_axis_entry, "Add all axes", boost::bind(&octomap_editing::OEServer::menuAddRemoveRotateAxisFeedback, this, _1));
    _mapping_menuentry_axis[rotate_axis_addAll_entry] = "addAll";

    interactive_markers::MenuHandler::EntryHandle rotate_axis_removeAll_entry = menu_handler.insert(rotate_axis_entry, "Remove all axes", boost::bind(&octomap_editing::OEServer::menuAddRemoveRotateAxisFeedback, this, _1));
    _mapping_menuentry_axis[rotate_axis_removeAll_entry] = "removeAll";

    return menu_handler;
  }

  void
  OEServer::menuAddRemoveMoveAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    std::shared_ptr<visualization_msgs::InteractiveMarker> marker = _markers[feedback->marker_name];

    if (_mapping_menuentry_axis[feedback->menu_entry_id] == "addAll")
    {
      marker->controls.push_back(_markerFactory.createMoveAxisControl("move_X"));
      marker->controls.push_back(_markerFactory.createMoveAxisControl("move_Y"));
      marker->controls.push_back(_markerFactory.createMoveAxisControl("move_Z"));
    }
    else
    {
      std::vector<visualization_msgs::InteractiveMarkerControl> new_controls;
      for (size_t i = 0; i < marker->controls.size(); ++i)
      {
        if (marker->controls[i].name == "")
        {
          new_controls.push_back(marker->controls[i]);
        }
      }
      marker->controls = new_controls;
    }
    refreshServer();
  }

  void
  OEServer:: menuAddRemoveRotateAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    std::shared_ptr<visualization_msgs::InteractiveMarker> marker = _markers[feedback->marker_name];

    if (_mapping_menuentry_axis[feedback->menu_entry_id] == "addAll")
    {
      marker->controls.push_back(_markerFactory.createRotateAxisControl("rotate_X"));
      marker->controls.push_back(_markerFactory.createRotateAxisControl("rotate_Y"));
      marker->controls.push_back(_markerFactory.createRotateAxisControl("rotate_Z"));
    }
    else
    {
      std::vector<visualization_msgs::InteractiveMarkerControl> new_controls;
      for (size_t i = 0; i < marker->controls.size(); ++i)
      {
        if (marker->controls[i].name == "")
        {
          new_controls.push_back(marker->controls[i]);
        }
      }
      marker->controls = new_controls;
    }
    refreshServer();
  }

  void
  OEServer::createCube()
  {
    // create the whole cube
    _cube = _markerFactory.createCube();
    std::vector<std::shared_ptr<OECubeMarker>> cube_markers = _cube.getCubeMarkers();
    for (auto it = cube_markers.begin(), end = cube_markers.end(); it != end; ++it)
    {
      _markers[(*it)->getMarker()->name] = (*it)->getMarker();
      _server.insert(*(*it)->getMarker(), boost::bind(&octomap_editing::OEServer::processCubeMarkerFeedback, this, _1));
    }
  }

  void
  OEServer::createControlMarker(std::string name)
  {
    // creates the interactive marker
    visualization_msgs::InteractiveMarker control_marker = _markerFactory.createControlMarker(name, 2, 2, 2);
    _markers[control_marker.name] = std::make_shared<visualization_msgs::InteractiveMarker>(control_marker);

    // creates the menu
    interactive_markers::MenuHandler menu_handler = createMenu();
    _menus[control_marker.name] = std::make_shared<interactive_markers::MenuHandler>(menu_handler);

    // insert marker with callback into the server
    _server.insert(*_markers[control_marker.name], boost::bind(&octomap_editing::OEServer::processMarkerFeedback, this, _1));

    // apply the menu handle to the control marker
    _menus[control_marker.name]->apply(_server, control_marker.name);
    _pointcloud_pose_change = control_marker.pose;

//    _imarker_pose_initial = _imarker.pose;
  }

  void
  OEServer::createTextMarker(std::string name)
  {
    visualization_msgs::InteractiveMarker text_marker = _markerFactory.createTextMarker(name);
    _markers[text_marker.name] = std::make_shared<visualization_msgs::InteractiveMarker>(text_marker);
    _server.insert(*_markers[text_marker.name]);
  }

  void
  OEServer::menuMoveAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    MenuHandler::CheckState check_state;
    std::shared_ptr<visualization_msgs::InteractiveMarker> marker = _markers[feedback->marker_name];
    std::shared_ptr<interactive_markers::MenuHandler> menu = _menus[marker->name];
    menu->getCheckState(feedback->menu_entry_id, check_state);

    if (check_state == MenuHandler::CHECKED)
    {
      menu->setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
      std::string name = "move_";
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
      marker->controls.push_back(_markerFactory.createMoveAxisControl("move_" + title));
    }
    refreshServer();
  }

  void
  OEServer::menuRotateAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
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
  OEServer::refreshServer()
  {
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
    _marker_pub.publish(lines);

    visualization_msgs::Marker triangles = _cube.getTriangles(_markerFactory.getNextSeq());
    _marker_pub.publish(triangles);

    for (auto it = _menus.begin(), end = _menus.end(); it != end; ++it)
    {
      (*(it->second)).reApply(_server);
    }
    // _server.setPose(_imarker_text.name, _imarker_text.pose); --> for the text marker!
    _server.applyChanges();
  }

  void
  OEServer::calculatePoseChange(geometry_msgs::Pose new_pose)
  {
    std::shared_ptr<visualization_msgs::InteractiveMarker> imarker = _markers["imarker_pointcloud"];
    _pointcloud_pose_change.position.x = (imarker->pose.position.x != new_pose.position.x) ? new_pose.position.x - imarker->pose.position.x : 0;
    _pointcloud_pose_change.position.y = (imarker->pose.position.y != new_pose.position.y) ? new_pose.position.y - imarker->pose.position.y : 0;
    _pointcloud_pose_change.position.z = (imarker->pose.position.z != new_pose.position.z) ? new_pose.position.z - imarker->pose.position.z : 0;

    _pointcloud_pose_change.orientation.w = 1;
    _pointcloud_pose_change.orientation.x = (imarker->pose.orientation.x != new_pose.orientation.x) ? new_pose.orientation.x - imarker->pose.orientation.x : 0;
    _pointcloud_pose_change.orientation.y = (imarker->pose.orientation.y != new_pose.orientation.y) ? new_pose.orientation.y - imarker->pose.orientation.y : 0;
    _pointcloud_pose_change.orientation.z = (imarker->pose.orientation.z != new_pose.orientation.z) ? new_pose.orientation.z - imarker->pose.orientation.z : 0;
  }


  void
  OEServer::processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
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
              // calculate the change in position
              calculatePoseChange(feedback->pose);
              publishPointCloud();
            }

            std::shared_ptr<visualization_msgs::InteractiveMarker> imarker = _markers[feedback->marker_name];
            imarker->pose = feedback->pose;

            updateText(feedback->pose);
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

    ocTree.writeBinary("/home/accname/programming/ROS/octomap_editing/fr_transformed.bt"); // TODO: name of saved tree
    std::cout << "Transformed OcTree saved!" << std::endl;
  }

  octomap::pose6d
  OEServer::getTransformationMatrix(bool transl)
  {
    if (transl)
    {
      octomath::Vector3 trans(static_cast<float>(_pointcloud_pose_change.position.x),
                               static_cast<float>(_pointcloud_pose_change.position.y),
                               static_cast<float>(_pointcloud_pose_change.position.z));
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
                                 static_cast<float>(_pointcloud_pose_change.orientation.x),
                                 static_cast<float>(_pointcloud_pose_change.orientation.y),
                                 static_cast<float>(_pointcloud_pose_change.orientation.z));
      octomap::pose6d transformMatrix(trans, rotat);
      return transformMatrix;
    }
  }

  void
  OEServer::updateText(geometry_msgs::Pose pose)
  {
    _server.erase("imarker_text");
    octomath::Vector3 v(pose.position.x, pose.position.y, pose.position.z);
    octomath::Quaternion q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    octomath::Pose6D p(v,q);

    std::shared_ptr<visualization_msgs::InteractiveMarker> imarker_text = _markers["imarker_text"];
    imarker_text->controls[0].markers[0].text = _markerFactory.formatText(p.x(), p.y(), p.z(), p.roll(), p.pitch(), p.yaw());
    imarker_text->pose = pose;
    _server.insert(*imarker_text);
    refreshServer();
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

    std::shared_ptr<visualization_msgs::InteractiveMarker> imarker = _markers["imarker_pointcloud"];


    calculatePoseChange(po);
    publishPointCloud();
    updateText(po);
    imarker->pose = po;

    refreshServer();
  }

  void
  OEServer::processCubeMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
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
          geometry_msgs::Point p1;
          p1.x = 0;
          p1.y = 0;
          p1.z = 0;

          geometry_msgs::Point p2;
          p2.x = 0;
          p2.y = 1;
          p2.z = 0;

          geometry_msgs::Point p3;
          p3.x = 1;
          p3.y = 0;
          p3.z = 0;

          geometry_msgs::Point p_c;
          p_c.x = 0.5;
          p_c.y = 0.5;
          p_c.z = -1;
          _cube.polygonstuff(p1, p2, p3, p_c);


          // test the walk trough the tree and the ispointinbox function
          /*uint inbox_counter = 0;
          for (auto it = _ocTree.begin_leafs(), end = _ocTree.end_leafs(); it != end; ++it)
          {
            if (_cube.isPointInBox(it.getCoordinate()))
            {
              ++inbox_counter;
            }
          }
          std::cout << "A total of " << inbox_counter << " points are inside the box!" << std::endl;*/
          refreshServer();
        }
        break;
    }
  }
}
