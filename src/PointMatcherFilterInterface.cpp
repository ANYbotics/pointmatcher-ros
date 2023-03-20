
#include "pointmatcher_ros/PointMatcherFilterInterface.h"

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
        ROS_ERROR_STREAM("Couldn't open pipeline description file from \"" << fileName << "\"");
        return false;
    }

    filters_ = Pm::DataPointsFilters(inFile);
    return true;
}

bool PointMatcherFilterInterface::readFiltersFromYamlNode(const YAML::Node& yamlNode)
{
    if (!yamlNode)
    {
        ROS_ERROR_STREAM("Couldn't read filters configuration.");
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
        ROS_WARN_STREAM_THROTTLE(10.0, "Caught exception: " << e.what() << ".  Point cloud unchanged. (Throttled: 10s)");
    }
}

} // namespace pointmatcher_ros