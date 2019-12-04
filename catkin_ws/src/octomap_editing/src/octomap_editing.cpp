#include <ros/ros.h>
#include <OEServer.hpp>
#include <octomap_server/OctomapServer.h>
#include <string>
#include <bitset>

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

  std::string mapFilename = std::string(argv[1]);
  octomap_editing::OEServer server(0.05, mapFilename, "octomap_editing");

  ros::spin();
  return 0;
}
