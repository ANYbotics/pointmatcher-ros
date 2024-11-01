
#include "pointmatcher_ros/PointMatcherFilterInterface.h"

#ifndef ROS2_BUILD
// ros
#include <ros/console.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif

// C++ standard library
#include <exception>
#include <fstream>

namespace pointmatcher_ros
{

bool PointMatcherFilterInterface::readPipelineFile(const std::string& fileName)
{
    std::ifstream inFile(fileName.c_str());
    if (!inFile.good())
    {
#ifndef ROS2_BUILD
        ROS_ERROR_STREAM("Couldn't open pipeline description file from \"" << fileName << "\"");
#else
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("PointmatcherRos"), "Couldn't open pipeline description file from \"" << fileName << "\"");
#endif
        return false;
    }

    filters_ = Pm::DataPointsFilters(inFile);
    return true;
}

bool PointMatcherFilterInterface::readFiltersFromYamlNode(const YAML::Node& yamlNode)
{
    if (!yamlNode)
    {
#ifndef ROS2_BUILD
        ROS_ERROR_STREAM("Couldn't read filters configuration.");
#else
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("PointmatcherRos"), "Couldn't read filters configuration.");
#endif
        return false;
    }
    filters_ = Pm::DataPointsFilters(yamlNode);
    return true;
}


Pm::DataPoints PointMatcherFilterInterface::process(const Pm::DataPoints& input)
{
    // Copy input point cloud.
    auto localInput = input;

    // Apply filters to point cloud.
    processInPlace(localInput);

    return localInput;
}

void PointMatcherFilterInterface::processInPlace(Pm::DataPoints& input)
{
    if (input.getNbPoints() == 0u)
    {
        // Point cloud is empty. Calling the filtering methods is unnecessary.
        return;
    }

    try
    {
        filters_.apply(input);
    }
    catch (const std::runtime_error& e)
    {
#ifndef ROS2_BUILD
        ROS_WARN_STREAM_THROTTLE(10.0, "Caught exception: " << e.what() << ".  Point cloud unchanged. (Throttled: 10s)");
#else
        // NOTE(apoghosov): we can not throttle here because the object lacks clock and creating it is expensive ...
        // it anyway does not make much sense because throwing exceptin above is already quite expensive ...
        RCLCPP_WARN_STREAM(rclcpp::get_logger("PointmatcherRos"),
                           "Caught exception: " << e.what() << ".  Point cloud unchanged. (Throttled: 10s)");
#endif
    }
}

} // namespace pointmatcher_ros