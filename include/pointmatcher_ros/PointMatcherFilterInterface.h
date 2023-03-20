#pragma once

// ros
#include <ros/console.h>

// PointMatcher
#include <pointmatcher/PointMatcher.h>

// PointMatcher ros
#include "pointmatcher_ros/usings.h"

namespace pointmatcher_ros
{

class PointMatcherFilterInterface
{
public:
    std::string getDataType() const { return dataType_; }
    void setDataType(std::string newValue) { dataType_ = newValue; }

    bool readPipelineFile(const std::string& fileName);

    /**
     * @brief Read the filters configuration from yaml node.
     * @param yamlNode The yaml node which contains the filters configuration.
     * @return True if the filters are configured properly. False otherwise.
     */
    bool readFiltersFromYamlNode(const YAML::Node& yamlNode);

    Pm::DataPoints process(const Pm::DataPoints& input);

    void processInPlace(Pm::DataPoints& input);

private:
    std::string dataType_;
    Pm::DataPointsFilters filters_;
};

} // namespace pointmatcher_ros