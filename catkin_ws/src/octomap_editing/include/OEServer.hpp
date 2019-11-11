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

namespace octomap_editing
{
  class OEServer : public octomap_server::OctomapServer
  {

  public:
    OEServer(ros::NodeHandle nh, double resolution = 0.1, std::string update_topic = "octomap_editing");

    void createInteractiveMarker(octomap::point3d center_coords);
    void createTextMarker(octomath::Pose6D pose);
    void openMapfile(std::string mapFilename);
    void testLineMarker();

  private:
    void createMenu();
    // menu callback functions
    void menuMoveAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void menuRotateAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void applyChangesFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void resetChangesFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void getPointCloudCallback(const sensor_msgs::PointCloud2 &pc);
    void publishPointCloud();
    octomap::pose6d getTransformationMatrix(bool transl);
    geometry_msgs::Pose calculatePoseChange();
    void updatePointCloud();
    void refreshServer(geometry_msgs::Pose pose);
    void updateText(geometry_msgs::Pose pose);

    interactive_markers::InteractiveMarkerServer _server;
    OEMarkerFactory _markerFactory;
    interactive_markers::MenuHandler _menu_handler;
    std::vector<interactive_markers::MenuHandler::EntryHandle> _menu_entries;
    visualization_msgs::InteractiveMarker _imarker;
    geometry_msgs::Pose _imarker_pose;
    geometry_msgs::Pose _imarker_pose_md;
    geometry_msgs::Pose _imarker_pose_mu;
    geometry_msgs::Pose _imarker_pose_change;
    geometry_msgs::Pose _imarker_pose_initial;
    visualization_msgs::InteractiveMarker _imarker_text;

    std::map<uint, char> _menuentry_to_axis_map;
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    ros::Publisher _pub;
    ros::Publisher _marker_pub;
    octomap::Pointcloud _ocPointcloud;
    octomap::ColorOcTree _ocTree;
  };
}
#endif // OESERVER_HPP
