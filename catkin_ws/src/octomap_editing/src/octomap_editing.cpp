#include <ros/ros.h>
#include <OEServer.hpp>
#include <octomap_server/OctomapServer.h>
#include <string>
#include <bitset>
#include <QApplication>
#include <OEPanel.hpp>
#include <thread>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#define USAGE "\nUSAGE: octomap_editing_node <map.[bt|ot]>\n" \
              "  map.bt: the octomap 3D map to edit\n"

/*
 * The function "startROS" is needed to start ROS.
 * QApplication's function exec() starts the displaying of the GUI.
 * ros::spin() is used to start ROS.
 * However if when both are started in the same thread, the ROS callbacks won't work anymore.
 * Therefore a new thread is started which simply starts ROS.
 */
void startROS()
{
  ros::spin();
}

int main(int argc, char** argv)
{
  if( !ros::isInitialized() )
  {
    ros::init(argc, argv, "octomap_editing");
  }

  if (argc != 2 || (argc == 2 && std::string(argv[1]) == "-h")){
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }

  // first start ROS
  std::string mapFilename = std::string(argv[1]);
  std::shared_ptr<octomap_editing::OEServer> server = std::make_shared<octomap_editing::OEServer>(0.05, mapFilename, "octomap_editing");
  std::thread thread_object(startROS);

  // needed for the Rviz Panel to save the changes made to the octomap
  QApplication app( argc, argv );
  octomap_editing::OEPanel* panel = new octomap_editing::OEPanel();
  panel->addServer(server);
  panel->show();
  app.exec();

  return 0;
}
