/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include "CameraPoseVisualization.h"
#include "../ros2_message_aliases.h"
#include "../ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "../estimator/estimator.h"
#include "../estimator/parameters.h"
#include <fstream>

extern rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr pub_odometry;
extern rclcpp::Publisher<nav_msgs::Path>::SharedPtr pub_path;
extern rclcpp::Publisher<visualization_msgs::Marker>::SharedPtr pub_key_poses;
extern nav_msgs::Path path;
extern int IMAGE_ROW, IMAGE_COL;

void registerPub(const rclcpp::Node::SharedPtr &node);

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t);

void pubTrackImage(const cv::Mat &imgTrack, const double t);

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header);

void pubInitialGuess(const Estimator &estimator, const std_msgs::Header &header);

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header);

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header);

void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header);

void pubTF(const Estimator &estimator, const std_msgs::Header &header);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);

void pubCar(const Estimator & estimator, const std_msgs::Header &header);
