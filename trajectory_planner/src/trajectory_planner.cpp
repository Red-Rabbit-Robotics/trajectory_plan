#include <ros/package.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "trajectory_planner/trajectory_planner.h"

// Replace the previous TOPPRA headers with these:
#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/parametrizer/const_accel.hpp>
#include <toppra/toppra.hpp>

bool getIK(const KDL::Chain& chain, const std::vector<std::pair<double, double>> joint_limits, const KDL::Frame& target_frame, KDL::JntArray& res) {
    KDL::JntArray lower_limits(chain.getNrOfJoints());
    KDL::JntArray upper_limits(chain.getNrOfJoints());
    for (size_t i = 0; i < joint_limits.size(); ++i) {
        lower_limits(i)=joint_limits[i].first;
        upper_limits(i)=joint_limits[i].second;
    }

    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::ChainIkSolverVel_pinv ik_solver_vel(chain);
    KDL::ChainIkSolverPos_NR_JL ik_solver(chain, lower_limits, upper_limits, fk_solver, ik_solver_vel, 100, 1e-5);

    KDL::JntArray q_init(chain.getNrOfJoints());
    KDL::JntArray q_out(chain.getNrOfJoints());

    int result = ik_solver.CartToJnt(q_init, target_frame, q_out);
    if (result < 0) {
        ROS_WARN("IK solver failed");
        return false; 
    }
   
    res = q_out;
    ROS_INFO("IK solver succeed");
    return true;
}

KDL::Frame poseToKDLFrame(const geometry_msgs::Pose& pose) {
    return KDL::Frame(
        KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        KDL::Vector(pose.position.x, pose.position.y, pose.position.z)
    );
}

TrajectoryPlanner::TrajectoryPlanner(const ros::NodeHandle& nh, const std::string& config_file)
    : nh_(nh),
      tf_buffer_(),
      tf_listener_(tf_buffer_),
      config_file_(config_file), 
      depth_camera_point_cloud_exist_(false), 
      cube_point_cloud_exist_(false),
      use_depth_camera_point_cloud_(true),
      depth_point_cloud_update_interval_(0.3),
      cube_point_cloud_update_interval_(5.0),
      plan_succeed_(false){
    
    loadConfig(config_file);

    depth_camera_sub_ = nh_.subscribe("/camera/depth/points", 1, &TrajectoryPlanner::depthCameraCallback, this);
    cube_obstacle_sub_ = nh_.subscribe("/cube_publisher/cube_point_cloud", 1, &TrajectoryPlanner::cubeObstacleCallback, this);
    left_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/left_arm_controller/command", 10);
    right_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/right_arm_controller/command", 10);
    visualize_joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("/visualize_joint_states", 10);
    left_joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("/left_arm_joint_states", 10);
    right_joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("/right_arm_joint_states", 10);
    combined_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/combined_point_cloud", 10);
    ompl_path_pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/ompl_path_pose_array", 10);

    plan_left_sub_ = nh_.subscribe("/plan_left_trajectory", 1, &TrajectoryPlanner::planLeftTrajectoryCallback, this);
    plan_right_sub_ = nh_.subscribe("/plan_right_trajectory", 1, &TrajectoryPlanner::planRightTrajectoryCallback, this);
    visualize_left_sub_ = nh_.subscribe("/visualize_left_trajectory", 1, &TrajectoryPlanner::visualizeLeftTrajectoryCallback, this);
    visualize_right_sub_ = nh_.subscribe("/visualize_right_trajectory", 1, &TrajectoryPlanner::visualizeRightTrajectoryCallback, this);
    publish_left_sub_ = nh_.subscribe("/publish_left_trajectory", 1, &TrajectoryPlanner::publishLeftTrajectoryCallback, this);
    publish_right_sub_ = nh_.subscribe("/publish_right_trajectory", 1, &TrajectoryPlanner::publishRightTrajectoryCallback, this);

    reload_config_srv_ = nh_.advertiseService("/reload_config", &TrajectoryPlanner::reloadConfigCallback, this);

    current_left_joint_angles_.resize(left_arm_.joint_names.size(), 0.0);
    current_right_joint_angles_.resize(right_arm_.joint_names.size(), 0.0);

    last_cube_point_cloud_update_time_ = ros::Time::now();
    last_depth_point_cloud_update_time_ = ros::Time::now();
}

bool TrajectoryPlanner::reloadConfigCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    loadConfig(config_file_);
    ROS_INFO("Configuration reloaded.");
    return true;
}

std::string resolvePackagePath(const std::string &path) {
    if (path.find("package://") == 0) {
        std::string package_name = path.substr(10, path.find('/', 10) - 10);
        std::string package_path = ros::package::getPath(package_name);
        if (package_path.empty()) {
            throw std::runtime_error("Package not found: " + package_name);
        }
        return package_path + path.substr(path.find('/', 10));
    }
    return path;
}
void TrajectoryPlanner::loadConfig(const std::string& config_file) {
    YAML::Node config = YAML::LoadFile(config_file);
    urdf_file_ = config["urdf_file"].as<std::string>();
    urdf_file_ = resolvePackagePath(urdf_file_);
    collision_radius_ = config["collision_radius"].as<double>();
    time_ = config["time"].as<double>();
    num_waypoints_ = config["num_waypoints"].as<int>();

    loadArmConfig(config["left_arm"], left_arm_);
    loadArmConfig(config["right_arm"], right_arm_);
}

void TrajectoryPlanner::loadArmConfig(const YAML::Node& arm_config, ArmConfig& arm) {
    arm.chain_start = arm_config["chain_start"].as<std::string>();
    arm.chain_end = arm_config["chain_end"].as<std::string>();
    arm.joint_names = arm_config["joint_names"].as<std::vector<std::string>>();

    urdf::Model model;
    if (!model.initFile(urdf_file_)) {
        ROS_ERROR("Failed to parse URDF file");
        throw std::runtime_error("Failed to parse URDF file");
    }

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromFile(urdf_file_, kdl_tree)) {
        ROS_ERROR("Failed to construct kdl tree");
        throw std::runtime_error("Failed to construct kdl tree");
    }

    if (!kdl_tree.getChain(arm.chain_start, arm.chain_end, arm.chain)) {
        ROS_ERROR("Failed to get kdl chain");
        throw std::runtime_error("Failed to get kdl chain");
    }

    arm.joint_limits = getJointLimits(model, arm.joint_names);
}

void TrajectoryPlanner::cubeObstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    //pcl::fromROSMsg(*msg, *cube_point_cloud_);

    pcl::fromROSMsg(*msg, cube_point_cloud_);
    
    cube_point_cloud_exist_ = true;


    if ((ros::Time::now() - last_cube_point_cloud_update_time_).toSec() > cube_point_cloud_update_interval_)
    {
        last_cube_point_cloud_update_time_ = ros::Time::now();
        if (depth_camera_point_cloud_exist_)
        {   
            combined_point_cloud_ = transformCloud(cube_point_cloud_, right_arm_.chain_start) + transformCloud(depth_camera_point_cloud_, right_arm_.chain_start);
        }
        else
        {
            combined_point_cloud_ = transformCloud(cube_point_cloud_, right_arm_.chain_start);
        }
        //combined_point_cloud_.header.frame_id = cube_point_cloud_.header.frame_id;
        // Debug combined point cloud
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(combined_point_cloud_, output);
        combined_point_cloud_pub_.publish(output);
    }
}

void TrajectoryPlanner::depthCameraCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if (use_depth_camera_point_cloud_ && (ros::Time::now() - last_depth_point_cloud_update_time_).toSec() > depth_point_cloud_update_interval_)
    {
        last_depth_point_cloud_update_time_ = ros::Time::now();

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
        
        cloud_transformed = transformCloud(cloud, right_arm_.chain_start);

        depth_camera_point_cloud_exist_ = true;
        // Downsample the point cloud for efficiency
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ> cloud_sampled;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud_transformed));
        vg.setInputCloud(cloud_ptr);
        vg.setLeafSize(0.05f, 0.05f, 0.05f);
        vg.filter(cloud_sampled);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud_sampled));
        //ROS_INFO("cloud_sampled size %d", cloud_sampled.points.size());
        //ROS_INFO("cloud_sampled ptr size %d", cloud_sampled_ptr->points.size());

        // Only keep points 1 meter away from the head_base_link frame
        pcl::PointCloud<pcl::PointXYZ> cloud_cropped;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud_cropped));
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_sampled_ptr);
        pass.setFilterFieldName("x"); //facing front
        pass.setFilterLimits(0, 1.2);
        pass.filter(*cloud_cropped_ptr);

        pass.setInputCloud(cloud_cropped_ptr);
        pass.setFilterFieldName("y"); //facing left
        pass.setFilterLimits(-1.5, 1.5);
        pass.filter(*cloud_cropped_ptr);

        pass.setInputCloud(cloud_cropped_ptr);
        pass.setFilterFieldName("z"); //facing up
        pass.setFilterLimits(-1.5, 0.5);
        pass.filter(*cloud_cropped_ptr);

        //ROS_INFO("cloud_cropped size %d", cloud_cropped.points.size());
        //ROS_INFO("cloud_cropped ptr size %d", cloud_cropped_ptr->points.size());

        // Filter out the arm itself
        // TODO: only filter out left arm when planning for right arm, vice versa
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        KDL::ChainFkSolverPos_recursive fk_solver_l(left_arm_.chain);
        KDL::ChainFkSolverPos_recursive fk_solver_r(right_arm_.chain);
        std::vector<KDL::Frame> link_frames_l(left_arm_.chain.getNrOfSegments());
        std::vector<KDL::Frame> link_frames_r(right_arm_.chain.getNrOfSegments());
        KDL::JntArray joint_positions_l(current_left_joint_angles_.size());
        KDL::JntArray joint_positions_r(current_right_joint_angles_.size());
        for (size_t i = 0; i < joint_positions_l.rows(); ++i)
        {
            joint_positions_l(i) = current_left_joint_angles_[i];
            joint_positions_r(i) = current_right_joint_angles_[i];
            //ROS_INFO("joint r[%d] angle: %f", i, current_right_joint_angles_[i]);
        }
        //fk_solver_l.JntToCart(joint_positions_l, link_frames_l);
        //fk_solver_r.JntToCart(joint_positions_r, link_frames_r);
        KDL::Frame l_frame, r_frame;
        for (size_t i = 0; i < left_arm_.chain.getNrOfSegments(); ++i)
        {
            fk_solver_l.JntToCart(joint_positions_l, l_frame, i+1);
            fk_solver_r.JntToCart(joint_positions_r, r_frame, i+1);
            link_frames_l[i]=l_frame;
            link_frames_r[i]=r_frame;
            //ROS_INFO("Right arm link frame: x: %f, y: %f, z: %f", r_frame.p.x(), r_frame.p.y(), r_frame.p.z());
        }

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        for (size_t i = 0; i < link_frames_l.size(); ++i) {
            KDL::Vector link_pos_l = link_frames_l[i].p;
            KDL::Vector link_pos_r = link_frames_r[i].p;
            //ROS_INFO("link_pos_r[%d] x:%f y:%f z:%f", i, link_pos_r.x(), link_pos_r.y(), link_pos_r.z());
            for (size_t j = 0; j < cloud_cropped_ptr->points.size(); ++j) {
                const auto& point = cloud_cropped_ptr->points[j];
                double distance_l = std::sqrt(std::pow(point.x - link_pos_l.x(), 2) +
                                            std::pow(point.y - link_pos_l.y(), 2) +
                                            std::pow(point.z - link_pos_l.z(), 2));
                double distance_r = std::sqrt(std::pow(point.x - link_pos_r.x(), 2) +
                                            std::pow(point.y - link_pos_r.y(), 2) +
                                            std::pow(point.z - link_pos_r.z(), 2));
               
                if (distance_l < 0.1 || distance_r < 0.1) {  // Adjust the threshold as needed
                    inliers->indices.push_back(j);
                }
            }
        }
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_cropped_ptr);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(cloud_filtered);

        //ROS_INFO("cloud_filtered size %d", cloud_filtered.points.size());
        
        depth_camera_point_cloud_ = cloud_filtered;
        depth_camera_point_cloud_.header = cloud_transformed.header;

        // Combine depth point cloud and cube point cloud 
        if (cube_point_cloud_exist_)
        {   
            combined_point_cloud_ = transformCloud(cube_point_cloud_, right_arm_.chain_start) + transformCloud(depth_camera_point_cloud_, right_arm_.chain_start);
        }
        else
        {
            combined_point_cloud_ = transformCloud(depth_camera_point_cloud_, right_arm_.chain_start);
        }
        //combined_point_cloud_.header.frame_id = depth_camera_point_cloud_.header.frame_id;
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(combined_point_cloud_, output);
        combined_point_cloud_pub_.publish(output);
    }
}

pcl::PointCloud<pcl::PointXYZ> TrajectoryPlanner::transformCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std::string& target_frame)
{
    if (cloud.header.frame_id == target_frame)
    {
        return cloud;
    }

    geometry_msgs::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_.lookupTransform(target_frame, cloud.header.frame_id, ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Transform warning: %s", ex.what());
    }

    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    pcl_ros::transformPointCloud(cloud, cloud_out, transform_stamped.transform);
    cloud_out.header.frame_id = target_frame;
    return cloud_out;
}

bool TrajectoryPlanner::planTrajectory(const ArmConfig& arm, const std::vector<double>& start_joint_angles, const geometry_msgs::Pose& target_pose, double time, int num_waypoints, trajectory_msgs::JointTrajectory& trajectory) {
    // Define the state space
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(arm.joint_names.size()));

    // Set the bounds for the state space
    ompl::base::RealVectorBounds bounds(arm.joint_names.size());
    for (size_t i = 0; i < arm.joint_names.size(); ++i) {
        bounds.setLow(i, arm.joint_limits[i].first);
        bounds.setHigh(i, arm.joint_limits[i].second);
    }
    space->setBounds(bounds);

    // Construct an instance of space information
    ompl::base::SpaceInformationPtr si(std::make_shared<ompl::base::SpaceInformation>(space));

    size_t dimension = si->getStateDimension();
    // Set state validity checking for this space
    si->setStateValidityChecker([this, dimension, arm](const ompl::base::State* state) {
        return isStateValid(state, arm.joint_limits, dimension);
    });

    // Create a problem instance
    ompl::geometric::SimpleSetup ss(si);

    // Set the start and goal states
    ompl::base::ScopedState<> start(space);
    ompl::base::ScopedState<> goal(space);

    for (size_t i = 0; i < arm.joint_names.size(); ++i) {
        start[i] = start_joint_angles[i];
    }

    KDL::Frame target_frame = poseToKDLFrame(target_pose);
    KDL::JntArray goal_joint_angles;
    if (!getIK(arm.chain, arm.joint_limits, target_frame, goal_joint_angles))
    {
        plan_succeed_ = false;
        return false;
    }

    for (size_t i = 0; i < arm.joint_names.size(); ++i) {
        goal[i] = goal_joint_angles(i);
    }

    ss.setStartAndGoalStates(start, goal);

    // Use the RRT* planner for optimization
    //auto planner(std::make_shared<ompl::geometric::RRTstar>(si));
    auto planner(std::make_shared<ompl::geometric::RRTXstatic>(si));
    ss.setPlanner(planner);

    // Set the optimization objective to minimize jerk
    ss.setOptimizationObjective(std::make_shared<JerkMinimizationObjective>(si));

    // Attempt to solve the problem within the given time
    ompl::base::PlannerStatus solved = ss.solve(5.0);

    trajectory.joint_names = arm.joint_names;

    if (solved) {
        ss.simplifySolution();
        ompl::geometric::PathGeometric ompl_path = ss.getSolutionPath();
        ompl_path.interpolate(num_waypoints);
      
        // Store poses for visualization
        geometry_msgs::PoseArray pose_array;
        pose_array.header.frame_id = "head_base_link";
        pose_array.header.stamp = ros::Time::now();
        auto arm_chain = right_arm_.chain;
        if(!planning_for_right_arm_)
        {
            arm_chain = left_arm_.chain;
        }
        KDL::ChainFkSolverPos_recursive fk_solver(arm_chain);  // Placeholder, update if necessary
        for (size_t i = 0; i < ompl_path.getStateCount(); ++i) {
            const auto *state = ompl_path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
            KDL::JntArray joint_positions(arm_chain.getNrOfJoints());
            for (size_t j = 0; j < arm_chain.getNrOfJoints(); ++j) {
                joint_positions(j) = state->values[j];
            }

            // Compute the forward kinematics to get the end effector pose
            KDL::Frame end_effector_pose;
            fk_solver.JntToCart(joint_positions, end_effector_pose);

            // Create a pose for the end effector
            geometry_msgs::Pose pose;
            pose.position.x = end_effector_pose.p.x();
            pose.position.y = end_effector_pose.p.y();
            pose.position.z = end_effector_pose.p.z();
            end_effector_pose.M.GetQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

            pose_array.poses.push_back(pose); 
        }
        ompl_path_pose_array_ = pose_array;

        std::vector<ompl::base::State*> states = ompl_path.getStates();

        // Time-parameterize the path using TOPPRA
        std::vector<Eigen::VectorXd> waypoints;
        for (const auto& state : states) {
            const auto* real_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
            Eigen::VectorXd waypoint(arm.joint_names.size());
            for (size_t j = 0; j < arm.joint_names.size(); ++j) {
                waypoint[j] = real_state->values[j];
            }
            waypoints.push_back(waypoint);
        }

        // Convert waypoints to Matrices type
        std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> matrices;
        for (const auto& waypoint : waypoints) {
            Eigen::MatrixXd matrix(waypoint.size(), 1);
            matrix.col(0) = waypoint;
            matrices.push_back(matrix);
        }

        // Create breakpoints (assuming uniform distribution for simplicity)
        std::vector<double> breakpoints(waypoints.size() + 1);
        for (size_t i = 0; i < breakpoints.size(); ++i) {
            breakpoints[i] = static_cast<double>(i) / (breakpoints.size() - 1);
        }

        // Create TOPPRA instance
        auto path = std::make_shared<toppra::PiecewisePolyPath>(matrices, breakpoints);

        // Set up constraints
        std::vector<toppra::LinearConstraintPtr> constraints;
        Eigen::VectorXd vel_limits = Eigen::VectorXd::Constant(path->dof(), 10.0);  // Adjust as needed
        Eigen::VectorXd acc_limits = Eigen::VectorXd::Constant(path->dof(), 10.0);  // Adjust as needed
        constraints.push_back(std::make_shared<toppra::constraint::LinearJointVelocity>(-vel_limits, vel_limits));
        constraints.push_back(std::make_shared<toppra::constraint::LinearJointAcceleration>(-acc_limits, acc_limits));

        // Create TOPPRA instance
        toppra::algorithm::TOPPRA algo(constraints, path);

        // Compute parameterization
        toppra::ReturnCode result = algo.computePathParametrization(0, 1);
        if (result == toppra::ReturnCode::OK) {
            // Generate trajectory points
            double total_time = time;
            int num_samples = 100;  // Adjust as needed
            for (int i = 0; i < num_samples; ++i) {
                double s_value = static_cast<double>(i) / (num_samples - 1);
                Eigen::VectorXd s(1);
                s << s_value;
                double t = s_value * total_time;

                Eigen::VectorXd position = path->eval(s)[0];
                Eigen::VectorXd velocity = path->eval(s, 1)[0];
                Eigen::VectorXd acceleration = path->eval(s, 2)[0];

                trajectory_msgs::JointTrajectoryPoint point;
                for (size_t j = 0; j < arm.joint_names.size(); ++j) {
                    point.positions.push_back(position[j]);
                    point.velocities.push_back(velocity[j]);
                    point.accelerations.push_back(acceleration[j]);
                }
                point.time_from_start = ros::Duration(t);
                trajectory.points.push_back(point);
            }
        } else {
            ROS_WARN("TOPPRA parameterization failed");
            return false;
        }
        return true;
    } else {
        ROS_WARN("Failed to plan the trajectory.");
        return false;
    }

}

void TrajectoryPlanner::planLeftTrajectoryCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    planning_for_right_arm_ = false;
    trajectory_msgs::JointTrajectory planned_trajectory;
    bool res= planTrajectory(left_arm_, current_left_joint_angles_, *msg, time_, num_waypoints_, planned_trajectory);
    
    if (res && !planned_trajectory.points.empty()) {
        left_planned_trajectory_ = planned_trajectory;
        plan_succeed_ = true;
        ROS_INFO("Left arm trajectory planned successfully.");
    } else {
        plan_succeed_ = false;
        ROS_WARN("Failed to plan the left arm trajectory.");
    }
}

void TrajectoryPlanner::planRightTrajectoryCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    planning_for_right_arm_ = true;
    trajectory_msgs::JointTrajectory planned_trajectory;
    bool res = planTrajectory(right_arm_, current_right_joint_angles_, *msg, time_, num_waypoints_, planned_trajectory);
    
    if (res && !planned_trajectory.points.empty()) {
        right_planned_trajectory_ = planned_trajectory;
        plan_succeed_ = true;
        ROS_INFO("Right arm trajectory planned successfully.");
    } else {
        plan_succeed_ = false;
        ROS_WARN("Failed to plan the right arm trajectory.");
    }
}

void TrajectoryPlanner::visualizeLeftTrajectoryCallback(const std_msgs::Empty::ConstPtr& msg) {
    if (plan_succeed_)
    {
        visualizeTrajectory(left_planned_trajectory_, current_left_joint_angles_);
        ROS_INFO("Left arm trajectory visualization complete.");
    }
}

void TrajectoryPlanner::visualizeRightTrajectoryCallback(const std_msgs::Empty::ConstPtr& msg) {
    if (plan_succeed_)
    {
        visualizeTrajectory(right_planned_trajectory_, current_right_joint_angles_);
        ROS_INFO("Right arm trajectory visualization complete.");
    }
}

void TrajectoryPlanner::visualizeTrajectory(const trajectory_msgs::JointTrajectory& planned_trajectory, std::vector<double>& current_joint_angles) {
    // Visualize the end effector trajectory
    ompl_path_pose_array_pub_.publish(ompl_path_pose_array_);
    
    // Visualize the arm movement 
    ros::Rate rate(30);
    for (const auto& point : planned_trajectory.points) {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = planned_trajectory.joint_names;
        joint_state.position = point.positions;
        joint_state.velocity = point.velocities;
        joint_state.effort = point.effort;  // Populate effort if available
        visualize_joint_states_pub_.publish(joint_state);
        rate.sleep();
    }
    sensor_msgs::JointState head_joint_state;
    /* 
    head_joint_state.header.stamp = ros::Time::now();
    head_joint_state.name = {"neck_yaw2pitch_joint"};
    head_joint_state.position = {448.0/2048.0*3.14}; //TODO use proper head joints value instead hardcoding
    head_joint_state.velocity = {0};
    head_joint_state.effort = {0};
    visualize_joint_states_pub_.publish(head_joint_state);
    */
    ros::Duration(3.0).sleep();  // Wait for 3 seconds

    // Move the robot back to the starting position
    for (const auto& point : planned_trajectory.points) {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = planned_trajectory.joint_names;
        joint_state.position = planned_trajectory.points.front().positions;  // Go back to initial positions
        joint_state.velocity = std::vector<double>(planned_trajectory.joint_names.size(), 0.0); // Zero velocities
        joint_state.effort = std::vector<double>(planned_trajectory.joint_names.size(), 0.0);   // Zero efforts
        visualize_joint_states_pub_.publish(joint_state);
        rate.sleep();
    }
    current_joint_angles = planned_trajectory.points.front().positions;  // Update current joint angles
}

void TrajectoryPlanner::publishLeftTrajectoryCallback(const std_msgs::Empty::ConstPtr& msg) {
    left_trajectory_pub_.publish(left_planned_trajectory_);
    trajectory_ = left_planned_trajectory_;
    current_left_joint_angles_ = left_planned_trajectory_.points.back().positions;  // Update current joint angles

    start_time_ = ros::Time::now();
    current_traj_point_ = 0;
    traj_to_joint_state_pub_timer_ = nh_.createTimer(ros::Duration(0.01), &TrajectoryPlanner::trajToJointStatePubTimerCallback, this);

    ROS_INFO("Left arm trajectory published successfully.");
}

void TrajectoryPlanner::publishRightTrajectoryCallback(const std_msgs::Empty::ConstPtr& msg) {
    right_trajectory_pub_.publish(right_planned_trajectory_);
    trajectory_ = right_planned_trajectory_;
    current_right_joint_angles_ = right_planned_trajectory_.points.back().positions;  // Update current joint angles
    
    start_time_ = ros::Time::now();
    current_traj_point_ = 0;
    traj_to_joint_state_pub_timer_ = nh_.createTimer(ros::Duration(0.01), &TrajectoryPlanner::trajToJointStatePubTimerCallback, this);
    ROS_INFO("Right arm trajectory published successfully.");
}


void TrajectoryPlanner::trajToJointStatePubTimerCallback(const ros::TimerEvent& event)
{
    if (current_traj_point_ >= trajectory_.points.size()) {
        traj_to_joint_state_pub_timer_.stop();
        return;
    }

    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time = current_time - start_time_;

    if (current_traj_point_ < trajectory_.points.size() &&
           elapsed_time >= trajectory_.points[current_traj_point_].time_from_start) {
        // Publish joint state
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = current_time;
        joint_state.name = trajectory_.joint_names;
        joint_state.position = trajectory_.points[current_traj_point_].positions;
        joint_state.velocity = trajectory_.points[current_traj_point_].velocities;
        joint_state.effort = trajectory_.points[current_traj_point_].effort;
       
        if (planning_for_right_arm_)
        {
            right_joint_states_pub_.publish(joint_state);
        }
        else
        {
            left_joint_states_pub_.publish(joint_state);
        }
        ++current_traj_point_;
    }
}

bool TrajectoryPlanner::isStateValid(const ompl::base::State* state, const std::vector<std::pair<double, double>> joint_limits, const size_t dimension) {
    // Enforce joint limits
    const auto *real_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
    for (size_t i = 0; i < joint_limits.size(); ++i) {
        if (real_state->values[i] < joint_limits[i].first || real_state->values[i] > joint_limits[i].second) {
            return false;
        }
    }

    // Enforce collision avoidance

    if (!depth_camera_point_cloud_exist_ && !cube_point_cloud_exist_)
    {
        return true;
    }

    if (combined_point_cloud_.points.size() == 0)
    {
        return true;
    }

    const auto* joint_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
    // Convert joint state to positions of all links
    //KDL::JntArray joint_positions(joint_state->as<ompl::base::RealVectorStateSpace::StateType>()->getDimension());
    KDL::JntArray joint_positions(dimension);
    for (size_t i = 0; i < joint_positions.rows(); ++i) {
        joint_positions(i) = joint_state->values[i];
    }

    auto arm_chain = right_arm_.chain;
    if(!planning_for_right_arm_)
    {
        arm_chain = left_arm_.chain;
    }

    KDL::ChainFkSolverPos_recursive fk_solver(arm_chain);  // Placeholder, update if necessary
    KDL::Frame link_pose;

    // Check for collisions with obstacles
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(combined_point_cloud_));
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(combined_point_cloud_ptr);

    for (size_t i = 0; i < right_arm_.chain.getNrOfSegments(); ++i) {
        fk_solver.JntToCart(joint_positions, link_pose, i);

        pcl::PointXYZ search_point;
        search_point.x = link_pose.p.x();
        search_point.y = link_pose.p.y();
        search_point.z = link_pose.p.z();
        std::vector<int> point_idx_radius_search;
        std::vector<float> point_radius_squared_distance;

        if (kdtree.radiusSearch(search_point, collision_radius_, point_idx_radius_search, point_radius_squared_distance) > 0) {
            return false; // Collision detected
        }
    }

    return true;
}

std::vector<std::pair<double, double>> TrajectoryPlanner::getJointLimits(const urdf::Model& model, const std::vector<std::string>& joint_names) {
    std::vector<std::pair<double, double>> joint_limits;
    for (const auto& joint_name : joint_names) {
        const auto& joint = model.getJoint(joint_name);
        if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC) {
            double lower = joint->limits->lower;
            double upper = joint->limits->upper;
            joint_limits.emplace_back(lower, upper);
        }
    }
    return joint_limits;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_planner");
    ros::NodeHandle nh;

    std::string config_file;
    std::string package_path = ros::package::getPath("trajectory_planner");
    std::string default_config_file = package_path + "/config/trajectory_planner.yaml";
    nh.param("config_file", config_file, default_config_file);
    TrajectoryPlanner planner(nh, config_file);

    ros::spin();

    return 0;
}

