#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "cube_publisher");
    ros::NodeHandle nh("~");
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cube_point_cloud", 1);

    // Parameters for the cube's pose and size
    double x, y, z, qx, qy, qz, qw, scale_x, scale_y, scale_z;
    nh.param("cube/x", x, 0.4);
    nh.param("cube/y", y, 0.0);
    nh.param("cube/z", z, 0.0);
    nh.param("cube/qx", qx, 0.0);
    nh.param("cube/qy", qy, 0.0);
    nh.param("cube/qz", qz, 0.0);
    nh.param("cube/qw", qw, 1.0);
    nh.param("cube/scale_x", scale_x, 0.2);
    nh.param("cube/scale_y", scale_y, 0.2);
    nh.param("cube/scale_z", scale_z, 0.4);

    ROS_INFO("y is %f", y);

    ros::Rate r(1);  // 1 Hz
    while (ros::ok()) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "head_base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "cube";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the cube
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = qx;
        marker.pose.orientation.y = qy;
        marker.pose.orientation.z = qz;
        marker.pose.orientation.w = qw;

        // Set the scale of the cube
        marker.scale.x = scale_x;
        marker.scale.y = scale_y;
        marker.scale.z = scale_z;

        // Set the color of the cube
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;

        marker.lifetime = ros::Duration();

        // Publish the marker
        marker_pub.publish(marker);

        // Create a point cloud for the obstacle
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (double dx = -scale_x / 2; dx <= scale_x / 2; dx += 0.05) {
            for (double dy = -scale_y / 2; dy <= scale_y / 2; dy += 0.05) {
                for (double dz = -scale_z / 2; dz <= scale_z / 2; dz += 0.05) {
                    pcl::PointXYZ point;
                    point.x = x + dx;
                    point.y = y + dy;
                    point.z = z + dz;
                    cloud.points.push_back(point);
                }
            }
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "head_base_link";
        output.header.stamp = ros::Time::now();

        // Publish the point cloud
        point_cloud_pub.publish(output);

        r.sleep();
    }

    return 0;
}

