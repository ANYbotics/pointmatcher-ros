#pragma once

#ifndef ROS2_BUILD
// ros
#include <ros/ros.h>

// geometry msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

// tf
#include <tf/tf.h>

// tf conversions
#include <tf_conversions/tf_eigen.h>
#else
// ros
#include <rclcpp/rclcpp.hpp>

// geometry msgs
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#endif

// pointmatcher_ros
#include "pointmatcher_ros/PmTf.h"
#include "pointmatcher_ros/usings.h"

namespace pointmatcher_ros
{

/*!
 * Creates a pointmatcher transformation from a ROS transform msg.
 * @param tfMsg ROS transform msg.
 * @return Pointmatcher transformation.
 */
#ifndef ROS2_BUILD
PmTf convertRosTfMsgToPmTf(const geometry_msgs::TransformStamped& tfMsg);
#else
PmTf convertRosTfMsgToPmTf(geometry_msgs::msg::TransformStamped const& tfMsg);
#endif

/*!
 * Creates a ROS transform msg from a pointmatcher transformation.
 * @param pmTf Pointmatcher transformation.
 * @return ROS transform msg.
 */
#ifndef ROS2_BUILD
geometry_msgs::TransformStamped convertPmTfToTfMsg(const PmTf& pmTf);
#else
geometry_msgs::msg::TransformStamped convertPmTfToTfMsg(PmTf const& pmTf);
#endif

/*!
 * Creates a pointmatcher transformation from a ROS transformation.
 * @param tf ROS transformation.
 * @return Pointmatcher transformation.
 */
#ifndef ROS2_BUILD
PmTf convertRosTfToPmTf(const tf::StampedTransform& tf);
#endif

/*!
 * Creates a ROS transformation from a pointmatcher transformation.
 * @param pmTf Pointmatcher transformation.
 * @return ROS transformation.
 */
#ifndef ROS2_BUILD
tf::StampedTransform convertPmTfToTf(const PmTf& pmTf);
#endif

/*!
 * Creates a pose from a ROS transformation.
 * @param tf ROS transformation.
 * @return Pose.
 */
#ifndef ROS2_BUILD
geometry_msgs::PoseStamped convertRosTfToRosTfMsg(const tf::StampedTransform& tf);
#endif

/*!
 * Creates a ROS transformation from a pose.
 * @param pose         Pose.
 * @param childFrameId Child frame id.
 * @return ROS transformation.
 */
#ifndef ROS2_BUILD
tf::StampedTransform convertPoseStampedMsgToRosTf(const geometry_msgs::PoseStamped& pose, const std::string& childFrameId);
#endif

/*!
 * Creates a pose from a pointmatcher transformation.
 * @param pmTf Pointmatcher transformation.
 * @return Pose.
 */
#ifndef ROS2_BUILD
geometry_msgs::PoseStamped convertPmTfToPose(const PmTf& pmTf);
#else
geometry_msgs::msg::PoseStamped convertPmTfToPose(PmTf const& pmTf);
#endif

/*!
 * Creates a pointmatcher transformation from a pose.
 * @param pose         Pose.
 * @param childFrameId Child frame id.
 * @return Pointmatcher transformation.
 */
#ifndef ROS2_BUILD
PmTf convertPoseStampedMsgToPmTf(const geometry_msgs::PoseStamped& pose, const std::string& childFrameId);
#else
PmTf convertPoseStampedMsgToPmTf(geometry_msgs::msg::PoseStamped const& pose, std::string const& childFrameId);
#endif


} // namespace pointmatcher_ros