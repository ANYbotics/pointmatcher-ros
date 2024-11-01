
#include "pointmatcher_ros/visualization_utils.h"

#ifndef ROS2_BUILD
#include <std_msgs/ColorRGBA.h>
#else
#include <std_msgs/msg/color_rgba.hpp>
#endif

namespace pointmatcher_ros
{

#ifndef ROS2_BUILD
std::optional<visualization_msgs::Marker> generateMarkersForSurfaceNormalVectors(const StampedPointCloud& pointCloud,
                                                                                 const ros::Time& timestamp,
#else
std::optional<visualization_msgs::msg::Marker> generateMarkersForSurfaceNormalVectors(
    const StampedPointCloud& pointCloud,
    const rclcpp::Time& timestamp,
#endif
                                                                                 const NormalVectorsMarkerGenerationParameters& parameters,
                                                                                 const RgbaColorMap::Values& color)
{
    if (pointCloud.isEmpty())
    {
        return {};
    }
    if (!pointCloud.dataPoints_.descriptorExists(parameters.pointCloudFieldId_))
    {
#ifndef ROS2_BUILD
        ROS_WARN("Point cloud has no normals descriptor. Publisher configuration or normal computation might be inconsistent.");
#else
        RCLCPP_WARN(rclcpp::get_logger("PointmatcherRos"),
                    "Point cloud has no normals descriptor. Publisher configuration or normal computation might be inconsistent.");
#endif
        return {};
    }

    /* Common values used for all visualizations */
    const PmDataPoints::Index numberOfPoints{ pointCloud.getSize() };
#ifndef ROS2_BUILD
    std_msgs::ColorRGBA colorMsg;
#else
    std_msgs::msg::ColorRGBA colorMsg;
#endif
    colorMsg.r = color[0];
    colorMsg.g = color[1];
    colorMsg.b = color[2];
    colorMsg.a = color[3];

    /* Vectors */
#ifndef ROS2_BUILD
    visualization_msgs::Marker vectorsMarker;
#else
    visualization_msgs::msg::Marker vectorsMarker;
#endif
    vectorsMarker.header.stamp = timestamp;
    vectorsMarker.header.frame_id = pointCloud.header_.frame_id;
    vectorsMarker.ns = parameters.vectorsMarkerNamespace_;
#ifndef ROS2_BUILD
    vectorsMarker.action = visualization_msgs::Marker::ADD;
    vectorsMarker.type = visualization_msgs::Marker::LINE_LIST;
#else
    vectorsMarker.action = visualization_msgs::msg::Marker::ADD;
    vectorsMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
#endif
    vectorsMarker.pose.orientation.w = 1.0;
    vectorsMarker.id = parameters.vectorsMarkerId_;
    vectorsMarker.scale.x = parameters.vectorsWidth_;
    vectorsMarker.color = colorMsg;
    vectorsMarker.points.resize(numberOfPoints * 2);

    const auto& surfaceNormalsView{ pointCloud.dataPoints_.getDescriptorViewByName(parameters.pointCloudFieldId_) };
    PmDataPoints::Index markerIndex{ 0 };
    for (PmDataPoints::Index pointIndex = 0; pointIndex < numberOfPoints; ++pointIndex)
    {
        // This loop leverages two indices:
        // - A point index, that goes through every point in the point cloud. It increments 1 step at a time.
        // - A marker index. As every normal line marker needs a start and end point, it increments 2 steps at a time.

        // The actual position of the point that the surface normal belongs to.
        vectorsMarker.points[markerIndex].x = pointCloud.dataPoints_.features(0, pointIndex);
        vectorsMarker.points[markerIndex].y = pointCloud.dataPoints_.features(1, pointIndex);
        vectorsMarker.points[markerIndex].z = pointCloud.dataPoints_.features(2, pointIndex);

        // End of arrow.
        vectorsMarker.points[markerIndex + 1].x =
            pointCloud.dataPoints_.features(0, pointIndex) + surfaceNormalsView(0, pointIndex) * parameters.vectorsScalingFactor_;
        vectorsMarker.points[markerIndex + 1].y =
            pointCloud.dataPoints_.features(1, pointIndex) + surfaceNormalsView(1, pointIndex) * parameters.vectorsScalingFactor_;
        vectorsMarker.points[markerIndex + 1].z =
            pointCloud.dataPoints_.features(2, pointIndex) + surfaceNormalsView(2, pointIndex) * parameters.vectorsScalingFactor_;
        markerIndex += 2;
    }

    return vectorsMarker;
}

#ifndef ROS2_BUILD
std::optional<visualization_msgs::MarkerArray> generateMarkersForSurfaceNormalPatches(
#else
std::optional<visualization_msgs::msg::MarkerArray> generateMarkersForSurfaceNormalPatches(
#endif
    const StampedPointCloud& pointCloud,
#ifndef ROS2_BUILD
    const ros::Time& timestamp,
#else
    const rclcpp::Time& timestamp,
#endif
    const NormalVectorsMarkerGenerationParameters& parameters,
    const RgbaColorMap::Values& color)
{
    if (pointCloud.isEmpty())
    {
        return {};
    }
    if (!pointCloud.dataPoints_.descriptorExists(parameters.pointCloudFieldId_))
    {
#ifndef ROS2_BUILD
        ROS_WARN("Point cloud has no normals descriptor. Publisher configuration or normal computation might be inconsistent.");
#else
        RCLCPP_WARN(rclcpp::get_logger("PointmatcherRos"),
                    "Point cloud has no normals descriptor. Publisher configuration or normal computation might be inconsistent.");
#endif
        return {};
    }

    /* Common values used for all visualizations */
    const std::size_t numberOfPoints{ pointCloud.getSize() };
#ifndef ROS2_BUILD
    std_msgs::ColorRGBA colorMsg;
#else
    std_msgs::msg::ColorRGBA colorMsg;
#endif
    colorMsg.r = color[0];
    colorMsg.g = color[1];
    colorMsg.b = color[2];
    colorMsg.a = color[3];

#ifndef ROS2_BUILD
    visualization_msgs::MarkerArray markerArray;
#else
    visualization_msgs::msg::MarkerArray markerArray;
#endif
    markerArray.markers.resize(numberOfPoints);

    const Eigen::Vector3f zAxis{ Eigen::Vector3f::UnitZ() };
    const auto& surfaceNormalsView{ pointCloud.dataPoints_.getDescriptorViewByName(parameters.pointCloudFieldId_) };
    for (size_t i = 0; i < numberOfPoints; ++i)
    {

        auto& marker{ markerArray.markers[i] };
        marker.header.stamp = timestamp;
        marker.header.frame_id = pointCloud.header_.frame_id;
        marker.ns = "leaf_planes";
#ifndef ROS2_BUILD
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CUBE;
#else
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::CUBE;
#endif
        marker.id = i;
        marker.color = colorMsg;

        // Set marker scale.
        if (surfaceNormalsView.col(i).isZero())
        {
            // TODO(ynava) Adapt to resolution, if contained in the point cloud.
            marker.scale.x = 0.16;
            marker.scale.y = 0.16;
            marker.scale.z = 0.01;
        }
        else
        {
            // TODO(ynava) Remove zero-normal points before falling into this loop.
            marker.scale.x = marker.scale.y = marker.scale.z = 0;
        }

        // Set position.
        marker.pose.position.x = pointCloud.dataPoints_.features(0, i);
        marker.pose.position.y = pointCloud.dataPoints_.features(1, i);
        marker.pose.position.z = pointCloud.dataPoints_.features(2, i);

        // Set orientation.
        const Eigen::Quaternionf q{ Eigen::Quaternionf::FromTwoVectors(zAxis, surfaceNormalsView.col(i)).normalized() };
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
    }

    return markerArray;
}


} // namespace pointmatcher_ros
