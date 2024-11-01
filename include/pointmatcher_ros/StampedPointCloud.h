#pragma once

// std
#include <memory>

#ifndef ROS2_BUILD
#include <ros/ros.h>

// std msgs
#include <std_msgs/Header.h>

// sensor_msgs
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#else
#include <rclcpp/rclcpp.hpp>

// std msgs
#include <std_msgs/msg/header.hpp>

// sensor_msgs
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#endif

// pointmatcher_ros
#include "pointmatcher_ros/PmTf.h"
#include "pointmatcher_ros/usings.h"

namespace pointmatcher_ros
{

class StampedPointCloud
{
public:
#ifndef ROS2_BUILD
    std_msgs::Header header_;
#else
    std_msgs::msg::Header header_;
#endif

    PmDataPoints dataPoints_;

protected:
    std::shared_ptr<PmTransformator> transformator_;

public:
    StampedPointCloud();

    // TODO(ynava) Move filesystem operations to separate file.
#ifndef ROS2_BUILD
    static StampedPointCloud FromFile(const std::string& filePath, const ros::Time& stamp, const std::string& frameId);
    bool fromFile(const std::string& filePath, const ros::Time& stamp, const std::string& frameId);
#else
    StampedPointCloud FromFile(std::string const& filePath, rclcpp::Time const& stamp, std::string const& frameId);
    bool fromFile(std::string const& filePath, rclcpp::Time const& stamp, std::string const& frameId);
#endif

    void toFile(const std::string& filePath) const;

    // TODO(ynava) Move ROS serialization functions to separate file.
#ifndef ROS2_BUILD
    static StampedPointCloud FromRosMsg(const sensor_msgs::PointCloud2& msg);
    void fromRosMsg(const sensor_msgs::PointCloud2& msg);
    sensor_msgs::PointCloud2 toRosMsg(const ros::Time stamp = ros::Time(0)) const;
    void toRosMsg(sensor_msgs::PointCloud2& msg) const;
#else
    static StampedPointCloud FromRosMsg(sensor_msgs::msg::PointCloud2 const& msg);
    void fromRosMsg(sensor_msgs::msg::PointCloud2 const& msg);
    sensor_msgs::msg::PointCloud2 toRosMsg(rclcpp::Time const& stamp = rclcpp::Time()) const;
    void toRosMsg(sensor_msgs::msg::PointCloud2& msg) const;
#endif

    StampedPointCloud createSimilarEmpty() const;

    bool isEmpty() const;
    unsigned int getSize() const;
    void clear();

    bool descriptorExists(const std::string& name) const;
    void addOneDimensionalDescriptor(const std::string& name, const float value);
    PmDataPointsView getDescriptorView(const std::string& name);
    PmDataPointsConstView getDescriptorConstView(const std::string& name) const;

    bool transform(const PmTf& tf);
    bool transform(const PmTfParameters& transform);

    // TODO(ynava) Move filtering methods to separate file.
    bool filter(PmPointCloudFilters& filters);
    bool filterByDistance(const float distanceThreshold, const bool keepInside);
    bool filterByThresholding(const std::string& descriptorName, const unsigned int& descriptorDimension, const float threshold,
                              const bool keepOverThreshold);

    bool add(const StampedPointCloud& other);

    bool splitByOverlap(const StampedPointCloud& other, const float distanceThreshold, PmMatches matches,
                        StampedPointCloud& otherOverlappingPoints, StampedPointCloud& otherNonOverlappingPoints) const;
    void splitByThresholding(const std::string& descriptorName, const unsigned int& descriptorDimension, const float threshold,
                             StampedPointCloud& pointsUnderThreshold, StampedPointCloud& pointsOverThreshold) const;

    unsigned int countPointsOverThreshold(const std::string& descriptorName, const float threshold) const;

    PmMatrix toSphericalCoordinates() const;
};

std::ostream& operator<<(std::ostream& ostream, const StampedPointCloud& pointCloud);

} // namespace pointmatcher_ros