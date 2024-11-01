#ifndef __POINTMATCHER_ROS_TRANSFORM_H
#define __POINTMATCHER_ROS_TRANSFORM_H

#ifndef ROS2_BUILD
// tf2_ros
#include <tf2_ros/transform_listener.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#endif

// pointmatcher
#include <pointmatcher/PointMatcher.h>

// pointmatcher_ros
#include "pointmatcher_ros/StampedPointCloud.h"

#ifndef ROS2_BUILD
namespace ros
{
struct Time;
}
#endif

namespace pointmatcher_ros
{
/**
 * @brief Transforms a point cloud to a target frame, using a fixed frame as reference.
 *
 * @param fixedFrame[in]  Fixed frame, to be used as reference for the motion between the point cloud and the target timestamp.
 * @param targetFrame[in] Target frame.
 * @param targetStamp[in] Target timestamp.
 * @param pointCloud[in/out]  Point cloud to transform.
 * @return true       If successul, false otherwise.
 */
#ifndef ROS2_BUILD
bool transformCloudToFrame(const std::string& fixedFrame, const std::string& targetFrame, const ros::Time& targetStamp,
                           const tf2_ros::Buffer& tfBuffer, StampedPointCloud& pointCloud, double waitTimeTfLookup = 0.1);
#else
bool transformCloudToFrame(std::string const& fixedFrame, std::string const& targetFrame, rclcpp::Time const& targetStamp,
                           tf2_ros::Buffer const& tfBuffer, StampedPointCloud& pointCloud, double waitTimeTfLookup = 0.1);
#endif

} // namespace pointmatcher_ros

#endif //__POINTMATCHER_ROS_TRANSFORM_H