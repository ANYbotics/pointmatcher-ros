#pragma once

// std
#include <string>

#ifndef ROS2_BUILD
// ros
#include <ros/ros.h>

// geometry msgs
#include <geometry_msgs/TransformStamped.h>

// tf
#include <tf/tf.h>
#else
// ros
#include <rclcpp/rclcpp.hpp>

// geometry msgs
#include <geometry_msgs/msg/transform_stamped.hpp>

// tf2
#include <tf2/LinearMath/Transform.h>
#endif

// pointmatcher_ros
#include "pointmatcher_ros/usings.h"

namespace pointmatcher_ros
{

class PmTf
{
public:
#ifndef ROS2_BUILD
    ros::Time stamp_{ 0 };
#else
    rclcpp::Time stamp_;
#endif
    std::string sourceFrameId_;
    std::string targetFrameId_;
    PmTfParameters parameters_{ Pm::Matrix::Identity(4, 4) };

protected:
    std::shared_ptr<PmTransformator> transformator_;

public:
    PmTf();

#ifndef ROS2_BUILD
    static PmTf FromRosTfMsg(const geometry_msgs::TransformStamped& tfMsg);
    void fromRosTfMsg(const geometry_msgs::TransformStamped& tfMsg);
    geometry_msgs::TransformStamped toRosTfMsg() const;
    void toRosTfMsg(geometry_msgs::TransformStamped& tfMsg) const;

    tf::StampedTransform toRosTf() const;
    void toRosTf(tf::StampedTransform& tf) const;
#else
    static PmTf FromRosTfMsg(geometry_msgs::msg::TransformStamped const& tfMsg);
    void fromRosTfMsg(geometry_msgs::msg::TransformStamped const& tfMsg);
    geometry_msgs::msg::TransformStamped toRosTfMsg() const;
    void toRosTfMsg(geometry_msgs::msg::TransformStamped& tfMsg) const;

    tf2::Transform toRosTf() const;
    void toRosTf(tf2::Transform& tf) const;
#endif

    PmTf inverse() const;
};

std::ostream& operator<<(std::ostream& ostream, const PmTf& tf);

} // namespace pointmatcher_ros
