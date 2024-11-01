#ifndef __POINTMATCHER_ROS_SERIALIZATION_POINT_CLOUD_H
#define __POINTMATCHER_ROS_SERIALIZATION_POINT_CLOUD_H

// pointmatcher
#include <pointmatcher/PointMatcher.h>

#ifndef ROS2_BUILD
// ros
#include <ros/time.h>

// sensor_msgs
#include <sensor_msgs/PointCloud2.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#endif

namespace pointmatcher_ros
{
template<typename T>
#ifndef ROS2_BUILD
sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg(const typename PointMatcher<T>::DataPoints& pmCloud, const std::string& frame_id,
                                                   const ros::Time& stamp);
#else
sensor_msgs::msg::PointCloud2 pointMatcherCloudToRosMsg(typename PointMatcher<T>::DataPoints const& pmCloud, std::string const& frame_id,
                                                        rclcpp::Time const& stamp);
#endif
} // namespace pointmatcher_ros

#endif //__POINTMATCHER_ROS_SERIALIZATION_POINT_CLOUD_H