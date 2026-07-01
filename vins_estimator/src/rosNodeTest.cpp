/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <map>
#include <mutex>
#include <queue>
#include <stdio.h>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "ros2_message_aliases.h"
#include "utility/visualization.h"

Estimator estimator;

std::queue<sensor_msgs::Imu::ConstSharedPtr> imu_buf;
std::queue<sensor_msgs::PointCloud::ConstSharedPtr> feature_buf;
std::queue<sensor_msgs::Image::ConstSharedPtr> img0_buf;
std::queue<sensor_msgs::Image::ConstSharedPtr> img1_buf;
std::mutex m_buf;

void img0_callback(const sensor_msgs::Image::ConstSharedPtr img_msg)
{
    std::lock_guard<std::mutex> lock(m_buf);
    img0_buf.push(img_msg);
}

void img1_callback(const sensor_msgs::Image::ConstSharedPtr img_msg)
{
    std::lock_guard<std::mutex> lock(m_buf);
    img1_buf.push(img_msg);
}

cv::Mat getImageFromMsg(const sensor_msgs::Image::ConstSharedPtr img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

void sync_process()
{
    while (rclcpp::ok())
    {
        if (STEREO)
        {
            cv::Mat image0, image1;
            double time = 0;
            {
                std::lock_guard<std::mutex> lock(m_buf);
                if (!img0_buf.empty() && !img1_buf.empty())
                {
                    double time0 = vins_ros2::stampToSec(img0_buf.front()->header.stamp);
                    double time1 = vins_ros2::stampToSec(img1_buf.front()->header.stamp);
                    if (time0 < time1 - 0.003)
                    {
                        img0_buf.pop();
                        printf("throw img0\n");
                    }
                    else if (time0 > time1 + 0.003)
                    {
                        img1_buf.pop();
                        printf("throw img1\n");
                    }
                    else
                    {
                        time = vins_ros2::stampToSec(img0_buf.front()->header.stamp);
                        image0 = getImageFromMsg(img0_buf.front());
                        img0_buf.pop();
                        image1 = getImageFromMsg(img1_buf.front());
                        img1_buf.pop();
                    }
                }
            }
            if (!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            double time = 0;
            {
                std::lock_guard<std::mutex> lock(m_buf);
                if (!img0_buf.empty())
                {
                    time = vins_ros2::stampToSec(img0_buf.front()->header.stamp);
                    image = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                }
            }
            if (!image.empty())
                estimator.inputImage(time, image);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void imu_callback(const sensor_msgs::Imu::ConstSharedPtr imu_msg)
{
    double t = vins_ros2::stampToSec(imu_msg->header.stamp);
    Eigen::Vector3d acc(
        imu_msg->linear_acceleration.x,
        imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z);
    Eigen::Vector3d gyr(
        imu_msg->angular_velocity.x,
        imu_msg->angular_velocity.y,
        imu_msg->angular_velocity.z);
    estimator.inputIMU(t, acc, gyr);
}

void feature_callback(const sensor_msgs::PointCloud::ConstSharedPtr feature_msg)
{
    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if (feature_msg->channels.size() > 8)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
    }
    double t = vins_ros2::stampToSec(feature_msg->header.stamp);
    estimator.inputFeature(t, featureFrame);
}

void restart_callback(const std_msgs::Bool::ConstSharedPtr restart_msg)
{
    if (restart_msg->data)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
}

void imu_switch_callback(const std_msgs::Bool::ConstSharedPtr switch_msg)
{
    estimator.changeSensorType(switch_msg->data ? 1 : 0, STEREO);
}

void cam_switch_callback(const std_msgs::Bool::ConstSharedPtr switch_msg)
{
    estimator.changeSensorType(USE_IMU, switch_msg->data ? 1 : 0);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("vins_estimator");

    node->declare_parameter<std::string>("config_file", "");
    std::string config_file = node->get_parameter("config_file").as_string();
    if (config_file.empty() && argc >= 2 && std::string(argv[1]).rfind("--", 0) != 0)
        config_file = argv[1];

    if (config_file.empty())
    {
        ROS_ERROR("Missing config_file parameter. Example: ros2 run vins vins_node --ros-args -p config_file:=/path/to/config.yaml");
        rclcpp::shutdown();
        return 1;
    }

    printf("config_file: %s\n", config_file.c_str());

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(node);

    rclcpp::Subscription<sensor_msgs::Imu>::SharedPtr sub_imu;
    if (USE_IMU)
    {
        sub_imu = node->create_subscription<sensor_msgs::Imu>(
            IMU_TOPIC, rclcpp::SensorDataQoS(), imu_callback);
    }
    auto sub_feature = node->create_subscription<sensor_msgs::PointCloud>(
        "/feature_tracker/feature", 2000, feature_callback);
    auto sub_img0 = node->create_subscription<sensor_msgs::Image>(
        IMAGE0_TOPIC, rclcpp::SensorDataQoS(), img0_callback);
    rclcpp::Subscription<sensor_msgs::Image>::SharedPtr sub_img1;
    if (STEREO)
    {
        sub_img1 = node->create_subscription<sensor_msgs::Image>(
            IMAGE1_TOPIC, rclcpp::SensorDataQoS(), img1_callback);
    }
    auto sub_restart = node->create_subscription<std_msgs::Bool>(
        "/vins_restart", 100, restart_callback);
    auto sub_imu_switch = node->create_subscription<std_msgs::Bool>(
        "/vins_imu_switch", 100, imu_switch_callback);
    auto sub_cam_switch = node->create_subscription<std_msgs::Bool>(
        "/vins_cam_switch", 100, cam_switch_callback);

    std::thread sync_thread{sync_process};
    rclcpp::spin(node);
    if (sync_thread.joinable())
        sync_thread.join();
    rclcpp::shutdown();
    return 0;
}
