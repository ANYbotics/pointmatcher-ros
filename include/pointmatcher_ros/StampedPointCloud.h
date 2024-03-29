#pragma once

// std
#include <memory>

// std msgs
#include <std_msgs/Header.h>

// sensor_msgs
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

// pointmatcher_ros
#include "pointmatcher_ros/PmTf.h"
#include "pointmatcher_ros/usings.h"

namespace pointmatcher_ros
{

class StampedPointCloud
{
public:
    std_msgs::Header header_;
    PmDataPoints dataPoints_;

protected:
    std::shared_ptr<PmTransformator> transformator_;

public:
    StampedPointCloud();

    // TODO(ynava) Move filesystem operations to separate file.
    static StampedPointCloud FromFile(const std::string& filePath, const ros::Time& stamp, const std::string& frameId);
    bool fromFile(const std::string& filePath, const ros::Time& stamp, const std::string& frameId);
    void toFile(const std::string& filePath) const;

    // TODO(ynava) Move ROS serialization functions to separate file.
    static StampedPointCloud FromRosMsg(const sensor_msgs::PointCloud2& msg);
    void fromRosMsg(const sensor_msgs::PointCloud2& msg);
    sensor_msgs::PointCloud2 toRosMsg(const ros::Time stamp = ros::Time(0)) const;
    void toRosMsg(sensor_msgs::PointCloud2& msg) const;

    StampedPointCloud createSimilarEmpty() const;

    bool isEmpty() const;
    unsigned int getSize() const;
    void clear();

    bool descriptorExists(const std::string& name) const;
    void addOneDimensionalDescriptor(const std::string& name, const float value);
    PmDataPointsView getDescriptorView(const std::string& name);
    PmDataPointsConstView getDescriptorConstView(const std::string& name) const;
    void setDescriptorFromDescriptor(const std::string& sourceDescriptorName, const std::string& targetDescriptorName);

    bool transform(const PmTf& tf);
    bool transform(const PmTfParameters& transform);

    // TODO(ynava) Move filtering methods to separate file.
    bool filter(PmPointCloudFilter& filter);
    bool filter(PmPointCloudFilters& filters);
    bool filterByDistance(const float distanceThreshold, const bool keepInside);
    bool filterByDistance(const float distanceThreshold, const bool keepInside, PmMatrix& newIdToOldId);
    bool filterByBoundingBox(const float minX, const float maxX, const float minY, const float maxY, const float minZ, const float maxZ,
                             const bool keepInside);
    bool filterByThresholding(const std::string& descriptorName, const unsigned int& descriptorDimension, const float threshold,
                              const bool keepOverThreshold);

    bool add(const StampedPointCloud& other);
    bool addNonOverlappingPoints(const StampedPointCloud& other, const float maxDistOverlappingPoints, PmMatches& matches);

    bool splitByOverlap(const StampedPointCloud& other, const float distanceThreshold, PmMatches matches,
                        StampedPointCloud& otherOverlappingPoints, StampedPointCloud& otherNonOverlappingPoints) const;
    void splitByThresholding(const std::string& descriptorName, const unsigned int& descriptorDimension, const float threshold,
                             StampedPointCloud& pointsUnderThreshold, StampedPointCloud& pointsOverThreshold) const;

    unsigned int countPointsOverThreshold(const std::string& descriptorName, const float threshold) const;

    PmMatrix toSphericalCoordinates() const;
};

std::ostream& operator<<(std::ostream& ostream, const StampedPointCloud& pointCloud);

} // namespace pointmatcher_ros