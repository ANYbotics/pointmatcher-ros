
#include "pointmatcher_ros/transform.h"

// ros
#ifndef ROS2_BUILD
#include <ros/time.h>

#include "pointmatcher_ros/PmTf.h"


// ugly test depending on roscpp because tf_conversions is not properly versionized
#if !ROS_VERSION_MINIMUM(1, 9, 30)
#define transformTFToEigen TransformTFToEigen
#define transformEigenToTF TransformEigenToTF
#endif // !ROS_VERSION_MINIMUM(1, 9, 30)
#endif

namespace pointmatcher_ros
{
#ifndef ROS2_BUILD
bool transformCloudToFrame(const std::string& fixedFrame, const std::string& targetFrame, const ros::Time& targetStamp,
                           const tf2_ros::Buffer& tfBuffer, StampedPointCloud& pointCloud, const double waitTimeTfLookup)
#else
bool transformCloudToFrame(std::string const& fixedFrame, std::string const& targetFrame, rclcpp::Time const& targetStamp,
                           tf2_ros::Buffer const& tfBuffer, StampedPointCloud& pointCloud, double waitTimeTfLookup)
#endif
{
    const auto& sourceFrame{ pointCloud.header_.frame_id };
    const auto& sourceStamp{ pointCloud.header_.stamp };

    if (sourceFrame.empty())
    {
#ifndef ROS2_BUILD
        ROS_WARN("Point cloud could not be transformed to target frame due to missing source frame.");
#else
        RCLCPP_WARN(rclcpp::get_logger("PointmatcherRos"),
                    "Point cloud could not be transformed to target frame due to missing source frame.");
#endif
        return false;
    }
    if (targetFrame.empty())
    {
#ifndef ROS2_BUILD
        ROS_WARN("Point cloud could not be transformed to target frame due to missing target frame.");
#else
        RCLCPP_WARN(rclcpp::get_logger("PointmatcherRos"),
                    "Point cloud could not be transformed to target frame due to missing target frame.");
#endif
        return false;
    }
    if (pointCloud.isEmpty())
    {
#ifndef ROS2_BUILD
        ROS_WARN("Point cloud could not be transformed to target frame as it contains no data.");
#else
        RCLCPP_WARN(rclcpp::get_logger("PointmatcherRos"), "Point cloud could not be transformed to target frame as it contains no data.");
#endif
        return false;
    }

    if (sourceFrame == targetFrame && sourceStamp == targetStamp)
    {
#ifndef ROS2_BUILD
        ROS_DEBUG("Early return when transforming point cloud as it is already in the target frame at the expected time.");
#else
        RCLCPP_DEBUG(rclcpp::get_logger("PointmatcherRos"),
                     "Early return when transforming point cloud as it is already in the target frame at the expected time.");
#endif
        return true;
    }

#ifndef ROS2_BUILD
    if (!tfBuffer.canTransform(targetFrame, targetStamp, sourceFrame, sourceStamp, fixedFrame, ros::Duration(waitTimeTfLookup)))
    {
        ROS_WARN_STREAM("Requested transform from frames \'" + sourceFrame + "' to '" + targetFrame + "', with target timestamp "
                        << targetStamp << " cannot be found.");
#else
    if (!tfBuffer.canTransform(targetFrame,
                               targetStamp,
                               sourceFrame,
                               sourceStamp,
                               fixedFrame,
                               rclcpp::Duration(std::chrono::duration<double>(waitTimeTfLookup))))
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("PointmatcherRos"),
                           "Requested transform from frames \'" << sourceFrame << "' to '" << targetFrame << "', with target timestamp "
                                                                << targetStamp.seconds() << " cannot be found.");
#endif
        return false;
    }

#ifndef ROS2_BUILD
    geometry_msgs::TransformStamped tfSourceToTargetRos;
#else
    geometry_msgs::msg::TransformStamped tfSourceToTargetRos;
#endif
    try
    {
        // Lookup tf from source frame to target frame, using an intermediate fixed frame
        tfSourceToTargetRos =
#ifndef ROS2_BUILD
            tfBuffer.lookupTransform(targetFrame, targetStamp, sourceFrame, sourceStamp, fixedFrame, ros::Duration(waitTimeTfLookup));
#else
            tfBuffer.lookupTransform(targetFrame,
                                     targetStamp,
                                     sourceFrame,
                                     sourceStamp,
                                     fixedFrame,
                                     rclcpp::Duration(std::chrono::duration<double>(waitTimeTfLookup)));
#endif
    }
    catch (const tf2::TransformException& exception)
    {
#ifndef ROS2_BUILD
        ROS_DEBUG_STREAM("Caught an exception while looking up transformation: " << exception.what());
#else
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("PointmatcherRos"),
                            "Caught an exception while looking up transformation: " << exception.what());
#endif
        return false;
    }

    // Transform point cloud from source frame to target frame.
    PmTf tfSourceToTarget = PmTf::FromRosTfMsg(tfSourceToTargetRos);
    pointCloud.transform(tfSourceToTarget);

    return true;
}

} // namespace pointmatcher_ros