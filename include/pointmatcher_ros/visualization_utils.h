#pragma once

#include <optional>
#include <string>

#ifndef ROS2_BUILD
#include <ros/time.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#else
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#endif

#include "pointmatcher_ros/Colors.h"
#include "pointmatcher_ros/StampedPointCloud.h"

namespace pointmatcher_ros
{

struct NormalVectorsMarkerGenerationParameters
{
    /* Vectors */
    //! ID of normals descriptor field.
    std::string pointCloudFieldId_{ "normals" };

    //! Marker namespace for normal vectors.
    std::string vectorsMarkerNamespace_{ "surface_normals_lines" };

    //! Marker ID for normal vectors.
    int vectorsMarkerId_{ 0 };

    //! Normal vectors scaling factor. For every normal expressed as a unitary vector, the scale factor allows conversion to an arbitrary scale.
    // For example, a factor of 0.1 allows normal visualizations with a length of 10cm.
    float vectorsScalingFactor_{ 0.1f };

    //! Width of normal vectors displayed.
    float vectorsWidth_{ 0.01f };

    // TODO(ynava) Parameters for representing the span of a surface normal patch.
};

/**
 * @brief Generates markers for a given set of surface normals vectors.
 * 
 * @param pointCloud    Point cloud to generate markers from. It is expected to contain 3D points and 3D normals.
 * @param timestamp     Timestamp to embed the markers message with.
 * @param parameters    Marker generation parameters.
 * @param color         RGB color for markers.
 * @return std::optional<visualization_msgs::Marker>    (Optional) marker representing surface normal vectors.
 */
#ifndef ROS2_BUILD
std::optional<visualization_msgs::Marker> generateMarkersForSurfaceNormalVectors(const StampedPointCloud& pointCloud,
                                                                                 const ros::Time& timestamp,
                                                                                 const NormalVectorsMarkerGenerationParameters& parameters,
                                                                                 const RgbaColorMap::Values& color);
#else
std::optional<visualization_msgs::msg::Marker> generateMarkersForSurfaceNormalVectors(
    StampedPointCloud const& pointCloud, rclcpp::Time const& timestamp, NormalVectorsMarkerGenerationParameters const& parameters,
    RgbaColorMap::Values const& color);
#endif

/**
 * @brief Generates tile markers for a given set of surface normals vectors.
 * 
 * @param pointCloud    Point cloud to generate markers from. It is expected to contain 3D points and 3D normals.
 * @param timestamp     Timestamp to embed the markers message with.
 * @param parameters    Marker generation parameters.
 * @param color         RGB color for markers.
 * @return std::optional<visualization_msgs::Marker>    (Optional) marker representing surface normal vectors as tiles
 */
#ifndef ROS2_BUILD
std::optional<visualization_msgs::MarkerArray> generateMarkersForSurfaceNormalPatches(
    const StampedPointCloud& pointCloud,
    const ros::Time& timestamp,
    const NormalVectorsMarkerGenerationParameters& parameters,
    const RgbaColorMap::Values& color);
#else
std::optional<visualization_msgs::msg::MarkerArray> generateMarkersForSurfaceNormalPatches(
    StampedPointCloud const& pointCloud, rclcpp::Time const& timestamp, NormalVectorsMarkerGenerationParameters const& parameters,
    RgbaColorMap::Values const& color);
#endif

} // namespace pointmatcher_ros