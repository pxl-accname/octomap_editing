#ifndef OESERVER_HPP
#define OESERVER_HPP

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <OEMarkerFactory.hpp>
#include <OECube.hpp>
#include <octomap_server/OctomapServer.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <map>


/*
 * This is the main class of the octomap_editing tool.
 * It creates the markers, updates them, saves the octomap etc.
 * Basically it manages the whole program.
 *
 */

namespace octomap_editing
{
  class OEServer : public octomap_server::OctomapServer
  {

  public:
    OEServer(double resolution, std::string mapFilename, std::string update_topic);
    void saveMap(bool delete_points);

    // marker functions
    void createControlMarker(std::string name = "");
    void createTextMarker(std::string name = "");
    void createCube();

  private:
    void openMapfile();
    interactive_markers::MenuHandler createControlMarkerMenu();
    void publishPointCloud();
    void refreshServer();
    void updateText();
    void calculatePoseChange(octomath::Pose6D pose);
    octomath::Pose6D getPClatestPose();
    octomath::Pose6D calculateLatestPoseChange();

    // callback functions
    void resetChangesCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void menuTranslationAxisCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void menuRotateAxisCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void processMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void menuAddRemoveTranslationAxisCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void menuAddRemoveRotateAxisCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void processCubeMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void getPointCloudCallback(const sensor_msgs::PointCloud2 &pc);

    // conversion functions
    octomath::Pose6D geometryPoseToPose6D(geometry_msgs::Pose geometryPose);
    geometry_msgs::Pose pose6DToGeometryPose(octomath::Pose6D octoPose);
    octomath::Pose6D getTotalTransformation();

    // helper functions
    std::string printCoords(octomath::Pose6D pose);

    ros::NodeHandle _nh;                                                                      // nodehandler of the package
    ros::Subscriber _sub_pc;                                                                  // subscriber to get the pointcloud from the octomap_server
    ros::Publisher _pub_pc;                                                                   // publishes the transformed pointcloud when the controll marker was moved
    octomap::Pointcloud _ocPointcloud;                                                        // stores the pointcloud received from the octomapserver
    ros::Publisher _pub_lines;                                                                // publishes the lines of the cube --> should this be in the cube class?
    std::map<std::string, std::shared_ptr<visualization_msgs::InteractiveMarker>> _markers;   // stores a pointer to all used interactive markers
    std::map<std::string, std::shared_ptr<interactive_markers::MenuHandler>> _menus;          // stores all the menus
    std::map<int, std::string> _mapping_menuentry_axis;                                       // stores mapping for the controll marker menu and the axis to manipulate
    std::string _mapFilename;                                                                 // stores the octomap filename
    interactive_markers::InteractiveMarkerServer _server;                                     // used to control the interactive markers
    OEMarkerFactory _markerFactory;                                                           // creates the markers
    std::vector<interactive_markers::MenuHandler::EntryHandle> _menu_entries;                 // handles the menus associated with the markers, currently only one menu
    std::vector<octomath::Pose6D> _history_pose;                                              // saves all poses of the pointcloud
    OECube _cube;                                                                             // stores the reference to the cube
    octomath::Pose6D _pc_cm_difference;                                                       // this variable stores the pose difference betwenn the pointcloud and the control marker
    float _resolution;                                                                        // stores the resolution of the octomap
  };
}
#endif // OESERVER_HPP
