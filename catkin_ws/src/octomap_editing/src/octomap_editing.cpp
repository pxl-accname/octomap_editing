#include <ros/ros.h>
#include <OEServer.hpp>
#include <octomap_server/OctomapServer.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#define USAGE "\nUSAGE: octomap_editing_node <map.[bt|ot]>\n" \
              "  map.bt: the octomap 3D map to edit\n"

int main(int argc, char** argv) {
  ros::init(argc, argv, "octomap_editing");

  if (argc != 2 || (argc == 2 && std::string(argv[1]) == "-h")){
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }

  // 1. step: get the filename
  std::string mapFilename = std::string(argv[1]);
  std::cout << "Mapfile: " << mapFilename << " found!" << std::endl;
  // 2. step: init the IO class

  // octomap_editing::OEIO OEIO();

  // 3. step: start the InteractiveMarkerServer
  ros::NodeHandle nh("octomap_editing");
  octomap_editing::OEServer server(nh, 1.0, "octomap_editing");
  server.openMapfile(mapFilename);

//  octomap_server::OctomapServer ocServer(nh);
//  std::cout << "Opening map file..." << std::endl;
//  ocServer.openFile(mapFilename);
//  std::cout << "Map file opened." << std::endl;


//  server.testLineMarker();
  octomap::point3d center(0., 0., 0.);
  octomath::Vector3 v(0,0,0);
  octomath::Quaternion q(1,0,0,0);
  octomath::Pose6D p6(v,q);
  server.createInteractiveMarker(center);
  server.createTextMarker(p6);

  ros::spin();
  return 0;
}
