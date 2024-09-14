#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <optional>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
//#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <urdf/model.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>
#include <std_srvs/Empty.h>
#include "trajectory_planner/JerkMinimizationObjective.h"

struct ArmConfig {
    std::string chain_start;
    std::string chain_end;
    std::vector<std::string> joint_names;
    KDL::Chain chain;
    std::vector<std::pair<double, double>> joint_limits;
};

class TrajectoryPlanner {
public:
    TrajectoryPlanner(const ros::NodeHandle& nh, const std::string& config_file);

    bool reloadConfigCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void loadConfig(const std::string& config_file);
    void loadArmConfig(const YAML::Node& arm_config, ArmConfig& arm);
    void depthCameraCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void cubeObstacleCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    pcl::PointCloud<pcl::PointXYZ> transformCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std::string& target_frame);
    bool planTrajectory(const ArmConfig& arm, const std::vector<double>& start_joint_angles, const geometry_msgs::Pose& target_pose, double time, int num_waypoints, trajectory_msgs::JointTrajectory& trajectory);
    void planLeftTrajectoryCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void planRightTrajectoryCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void visualizeLeftTrajectoryCallback(const std_msgs::Empty::ConstPtr& msg);
    void visualizeRightTrajectoryCallback(const std_msgs::Empty::ConstPtr& msg);
    void visualizeTrajectory(const trajectory_msgs::JointTrajectory& planned_trajectory, std::vector<double>& current_joint_angles);
    void publishLeftTrajectoryCallback(const std_msgs::Empty::ConstPtr& msg);
    void publishRightTrajectoryCallback(const std_msgs::Empty::ConstPtr& msg);
    void trajToJointStatePubTimerCallback(const ros::TimerEvent& event);

private:
    bool isStateValid(const ompl::base::State* state, const std::vector<std::pair<double, double>> joint_limits, const size_t dimension);
    std::vector<std::pair<double, double>> getJointLimits(const urdf::Model& model, const std::vector<std::string>& joint_names);

    ros::NodeHandle nh_;
    ArmConfig left_arm_;
    ArmConfig right_arm_;
    std::vector<double> current_left_joint_angles_;
    std::vector<double> current_right_joint_angles_;
    double collision_radius_;
    double time_;
    int num_waypoints_;
    std::string urdf_file_;
    std::string config_file_;

    //ros::Subscriber point_cloud_sub_;
    ros::Subscriber cube_obstacle_sub_;
    ros::Subscriber depth_camera_sub_;
    ros::Publisher left_trajectory_pub_;
    ros::Publisher right_trajectory_pub_;
    ros::Publisher visualize_joint_states_pub_;
    ros::Publisher left_joint_states_pub_;
    ros::Publisher right_joint_states_pub_;
    ros::Publisher combined_point_cloud_pub_;
    ros::Publisher ompl_path_pose_array_pub_;
    ros::Subscriber plan_left_sub_;
    ros::Subscriber plan_right_sub_;
    ros::Subscriber visualize_left_sub_;
    ros::Subscriber visualize_right_sub_;
    ros::Subscriber publish_left_sub_;
    ros::Subscriber publish_right_sub_;
    ros::ServiceServer reload_config_srv_;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
    pcl::PointCloud<pcl::PointXYZ> cube_point_cloud_;
    pcl::PointCloud<pcl::PointXYZ> depth_camera_point_cloud_;
    pcl::PointCloud<pcl::PointXYZ> combined_point_cloud_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    trajectory_msgs::JointTrajectory left_planned_trajectory_;
    trajectory_msgs::JointTrajectory right_planned_trajectory_;
    geometry_msgs::PoseArray ompl_path_pose_array_; 

    bool use_depth_camera_point_cloud_;
    bool depth_camera_point_cloud_exist_;
    bool cube_point_cloud_exist_;
    double depth_point_cloud_update_interval_;
    double cube_point_cloud_update_interval_;
    ros::Time last_depth_point_cloud_update_time_;
    ros::Time last_cube_point_cloud_update_time_;

    bool planning_for_right_arm_;

    trajectory_msgs::JointTrajectory trajectory_;
    ros::Timer traj_to_joint_state_pub_timer_;
    size_t current_traj_point_;
    ros::Time start_time_;

    bool plan_succeed_;
};

#endif // TRAJECTORY_PLANNER_H

