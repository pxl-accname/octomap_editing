#ifndef OESERVER_HPP
#define OESERVER_HPP

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <OEMarker.hpp>
#include <OEMarkerFactory.hpp>
#include <pcl_ros/point_cloud.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <pcl_ros/transforms.h>
#include <octomap_server/OctomapServer.h>
#include <octomap_ros/conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <OECube.hpp>
#include <map>


namespace octomap_editing
{
  class OEServer : public octomap_server::OctomapServer
  {

  public:
    OEServer(double resolution, std::string mapFilename, std::string update_topic);
    void createControlMarker(std::string name = "");
    void createTextMarker(std::string name = "");
    void createCube();

  private:
    // reorganized functions
    void openMapfile();



    interactive_markers::MenuHandler createMenu();
    // menu callback functions
    void menuMoveAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void menuRotateAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void applyChangesFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void resetChangesFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void menuAddRemoveMoveAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void menuAddRemoveRotateAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void processCubeMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void getPointCloudCallback(const sensor_msgs::PointCloud2 &pc);
    void publishPointCloud();
    octomap::pose6d getTransformationMatrix(bool transl);
    void refreshServer();
    void updateText(geometry_msgs::Pose pose);
    void calculatePoseChange(geometry_msgs::Pose pose);

    interactive_markers::InteractiveMarkerServer _server;
    OEMarkerFactory _markerFactory;
    std::vector<interactive_markers::MenuHandler::EntryHandle> _menu_entries;
    visualization_msgs::InteractiveMarker _imarker;
    geometry_msgs::Pose _pointcloud_pose_change;
    visualization_msgs::InteractiveMarker _imarker_text;

    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    ros::Publisher _pub;
    ros::Publisher _marker_pub;
    octomap::Pointcloud _ocPointcloud;
    octomap::ColorOcTree _ocTree;

    // reorganisation of the class ######################################
    std::string _mapFilename;
    std::map<std::string, std::shared_ptr<visualization_msgs::InteractiveMarker>> _markers;
    std::map<std::string, std::shared_ptr<interactive_markers::MenuHandler>> _menus;
    std::map<int, std::string> _mapping_menuentry_axis;
    OECube _cube;
  };
}
#endif // OESERVER_HPP
