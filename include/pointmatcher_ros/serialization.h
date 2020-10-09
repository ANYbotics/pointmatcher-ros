#ifndef __POINTMATCHER_ROS_SERIALIZATION_POINT_CLOUD_H
#define __POINTMATCHER_ROS_SERIALIZATION_POINT_CLOUD_H

// pointmatcher
#include <pointmatcher/PointMatcher.h>

// ros
#include <ros/time.h>

// sensor_msgs
#include <sensor_msgs/PointCloud2.h>

namespace PointMatcher_ros
{
	template<typename T>
	sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg(const typename PointMatcher<T>::DataPoints& pmCloud, const std::string& frame_id, const ros::Time& stamp);
} // PointMatcher_ros

#endif //__POINTMATCHER_ROS_SERIALIZATION_POINT_CLOUD_H