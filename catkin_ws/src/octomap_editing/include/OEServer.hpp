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

/*
 * This class loads the same .bt file as the server which is publishing the pointcloud etc.
 * It does not seem to be that easy to access a procted member variable from an base class...
 * Is there a way?
 *
 * Maybe remove the inheritance and copy some functionality from the OctomapServer class directly.
 */



namespace octomap_editing
{
  class OEServer : public octomap_server::OctomapServer
  {

  public:
    OEServer(double resolution, std::string mapFilename, std::string update_topic);
    void createControlMarker(std::string name = "");
    void createTextMarker(std::string name = "");
    void createCube();
    void saveMap(bool delete_points);

  private:
    // reorganized functions
    void openMapfile();

    // conversion functions
    octomath::Pose6D geometryPoseToPose6D(geometry_msgs::Pose geometryPose);
    geometry_msgs::Pose pose6DToGeometryPose(octomath::Pose6D octoPose);
    octomath::Pose6D getTotalTransformation();
    std::string printCoords(octomath::Pose6D pose);

    interactive_markers::MenuHandler createMenu();
    // menu callback functions
    void menuMoveAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void menuRotateAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void resetChangesFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void menuAddRemoveMoveAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void menuAddRemoveRotateAxisFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void processCubeMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void getPointCloudCallback(const sensor_msgs::PointCloud2 &pc);
    void publishPointCloud();
//    octomap::pose6d getTransformationMatrix(bool transl);
    void refreshServer();
    void updateText();
    void calculatePoseChange(octomath::Pose6D pose);

    octomath::Pose6D getPClatestPose();
    octomath::Pose6D calculateLatestPoseChange();


    interactive_markers::InteractiveMarkerServer _server;                     // used to serve the interactive markers
    OEMarkerFactory _markerFactory;                                           // creates the markers
    std::vector<interactive_markers::MenuHandler::EntryHandle> _menu_entries; // needed to handle the menus associated with the markers
    visualization_msgs::InteractiveMarker _imarker;                           // that should be the control marker
    geometry_msgs::Pose _pointcloud_pose_change;                              // total change of the pointcloud --> can be obtained by combining all pose6d in the history
    visualization_msgs::InteractiveMarker _imarker_text;                      // the test shown by the control marker
    std::vector<octomath::Pose6D> _history_pose;                              // saves all positions of the pointcloud --> needed for a history function (undo/redo)
    octomath::Pose6D _pc_cm_difference;

    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    ros::Publisher _pub;
    ros::Publisher _marker_pub;
    octomap::Pointcloud _ocPointcloud;

    // reorganisation of the class ######################################
    std::string _mapFilename;
    std::map<std::string, std::shared_ptr<visualization_msgs::InteractiveMarker>> _markers;
    std::map<std::string, std::shared_ptr<interactive_markers::MenuHandler>> _menus;
    std::map<int, std::string> _mapping_menuentry_axis;
    OECube _cube;

    // test
    ros::Publisher _marker_planes;
    ros::Publisher _marker_lines;
    ros::Publisher _marker_result;
  };
}
#endif // OESERVER_HPP
