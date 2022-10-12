
#include "pointmatcher_ros/visualization_utils.h"

#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace pointmatcher_ros
{

std::optional<visualization_msgs::Marker> generateMarkersForSurfaceNormalVectors(const StampedPointCloud& pointCloud,
                                                                                 const ros::Time& timestamp,
                                                                                 const NormalVectorsMarkerGenerationParameters& parameters,
                                                                                 const RgbaColorMap::Values& color)
{
    if (pointCloud.isEmpty())
    {
        return {};
    }
    if (!pointCloud.dataPoints_.descriptorExists(parameters.pointCloudFieldId_))
    {
        ROS_WARN("Point cloud has no normals descriptor. Publisher configuration or normal computation might be inconsistent.");
        return {};
    }

    /* Common values used for all visualizations */
    const PmDataPoints::Index numberOfPoints{ pointCloud.getSize() };
    std_msgs::ColorRGBA colorMsg;
    colorMsg.r = color[0];
    colorMsg.g = color[1];
    colorMsg.b = color[2];
    colorMsg.a = color[3];

    /* Vectors */
    visualization_msgs::Marker vectorsMarker;
    vectorsMarker.header.stamp = timestamp;
    vectorsMarker.header.frame_id = pointCloud.header_.frame_id;
    vectorsMarker.ns = parameters.vectorsMarkerNamespace_;
    vectorsMarker.action = visualization_msgs::Marker::ADD;
    vectorsMarker.type = visualization_msgs::Marker::LINE_LIST;
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


} // namespace pointmatcher_ros