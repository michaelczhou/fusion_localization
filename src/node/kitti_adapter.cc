//
// Created by zhouchang on 19-4-15.
//
#include "../utility/kitti_adapter.hpp"

#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <iomanip>
#include <istream>
#include <sstream>
#include <streambuf>

#include "../../external/loguru.hpp"

using namespace loam;

ros::Publisher pub_laser_cloud;
ros::Publisher pub_wheel;

void OXTSHandler(const double* data, int data_size, long s, long ns) {
    geometry_msgs::TwistStamped wheel_msg;
    wheel_msg.header.frame_id = "/imu_link";
    wheel_msg.header.stamp = ros::Time().fromSec((double)s + 1e-9*(double)ns);
    wheel_msg.twist.linear.x = data[8];
    wheel_msg.twist.linear.y = data[9];
    wheel_msg.twist.linear.z = data[10];
    wheel_msg.twist.angular.x = data[20];
    wheel_msg.twist.angular.y = data[21];
    wheel_msg.twist.angular.z = data[22];
    pub_wheel.publish(wheel_msg);
}

void laserHandler(const double* data, int data_size, long s, long ns) {
    pcl::PointCloud<pcl::PointXYZI> laser_cloud;
    pcl::PointXYZI point;
    for (int i = 0; i < data_size; i += 4 ) {
        point.x = data[i + 0];
        point.y = data[i + 1];
        point.z = data[i + 2];
        point.intensity = data[i + 3];
        laser_cloud.push_back(point);
    }

    sensor_msgs::PointCloud2 laser_cloud_msg;
    pcl::toROSMsg(laser_cloud, laser_cloud_msg);
    laser_cloud_msg.header.stamp = ros::Time().fromSec((double)s + 1e-9*(double)ns);
    laser_cloud_msg.header.frame_id = "/imu_link";
    pub_laser_cloud.publish(laser_cloud_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti_adapter");
    ros::NodeHandle n("~");
    std::string dataset_folder, calib_folder, output_bag_file;
    n.getParam("dataset_folder", dataset_folder);
    n.getParam("calib_folder", calib_folder);

    bool to_bag;
    n.getParam("to_bag", to_bag);
    if (to_bag)
        n.getParam("output_bag_file", output_bag_file);

    pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud", 2);
    pub_wheel = n.advertise<geometry_msgs::TwistStamped>("/wheel",200);

    rosbag::Bag bag_out;
    if (to_bag)
        bag_out.open(output_bag_file, rosbag::bagmode::Write);

    KITTIAdapter adapter(dataset_folder, calib_folder);

    adapter.OnNextOXTS(OXTSHandler);
    adapter.OnNextVelodyne(laserHandler);

    bool has_data = true;
    while (has_data) {
        has_data = adapter.NextSensor();
    }

    bag_out.close();
    std::cout << "Done \n";
}
