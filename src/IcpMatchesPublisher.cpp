
#include "pointmatcher_ros/IcpMatchesPublisher.h"

#ifndef ROS2_BUILD
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/PointCloud2.h>
#endif

namespace pointmatcher_ros
{

IcpMatchesPublisher::IcpMatchesPublisher() = default;

#ifndef ROS2_BUILD
void IcpMatchesPublisher::advertiseFromRosParameters(ros::NodeHandle nodeHandle, const std::string& parametersKey)
{
    if (!nodeHandle.getParam("publishers/" + parametersKey + "/reading_topic", parameters_.readingPointCloudPublisherTopic_))
    {
        ROS_ERROR("Topic name for point cloud publisher could not be fetched from the ROS Parameter Server.");
        return;
    }
    if (!nodeHandle.getParam("publishers/" + parametersKey + "/reference_topic", parameters_.referencePointCloudPublisherTopic_))
    {
        ROS_ERROR("Topic name for point cloud publisher could not be fetched from the ROS Parameter Server.");
        return;
    }
    parameters_.queueSizeOfPublishers_ = nodeHandle.param("publishers/" + parametersKey + "/queue_size", false);
    parameters_.latchPublishers_ = nodeHandle.param("publishers/" + parametersKey + "/latch", false);
    parameters_.publishSurfaceNormals_ = nodeHandle.param("publishers/" + parametersKey + "/surface_normals", false);
    parameters_.readingMarkersColor_ = static_cast<ColorKey>(
        nodeHandle.param("publishers/" + parametersKey + "/reading_surface_normals_color", static_cast<int>(ColorKey::kWhite)));
    parameters_.referenceMarkersColor_ = static_cast<ColorKey>(
        nodeHandle.param("publishers/" + parametersKey + "/reference_surface_normals_color", static_cast<int>(ColorKey::kWhite)));
#else
void IcpMatchesPublisher::advertiseFromRosParameters(rclcpp::Node::SharedPtr const& nodeHandle, std::string const& parametersKey)
{
    bool success;
    try
    {
        success = nodeHandle->get_parameter("publishers." + parametersKey + ".reading_topic", parameters_.readingPointCloudPublisherTopic_)
            && nodeHandle->get_parameter("publishers." + parametersKey + ".reference_topic",
                                         parameters_.referencePointCloudPublisherTopic_);
    }
    catch (rclcpp::exceptions::InvalidParameterTypeException const&)
    {
        success = false;
    }
    if (!success)
    {
        RCLCPP_ERROR(rclcpp::get_logger("PointmatcherRos"),
                     "Topic name for point cloud publisher could not be fetched from the ROS Parameter Server.");
        return;
    }
    try
    {
        parameters_.queueSizeOfPublishers_ = nodeHandle->get_parameter_or("publishers." + parametersKey + ".queue_size", 0);
        parameters_.latchPublishers_ = nodeHandle->get_parameter_or("publishers." + parametersKey + ".latch", false);
        parameters_.publishSurfaceNormals_ = nodeHandle->get_parameter_or("publishers." + parametersKey + ".surface_normals", false);
        parameters_.readingMarkersColor_ = static_cast<ColorKey>(nodeHandle->get_parameter_or(
            "publishers." + parametersKey + ".reading_surface_normals_color", static_cast<int>(ColorKey::kWhite)));
        parameters_.referenceMarkersColor_ = static_cast<ColorKey>(nodeHandle->get_parameter_or(
            "publishers." + parametersKey + ".reference_surface_normals_color", static_cast<int>(ColorKey::kWhite)));
    }
    catch (rclcpp::exceptions::InvalidParameterTypeException const&)
    {
        RCLCPP_ERROR(rclcpp::get_logger("PointmatcherRos"), "Some of the parameters have values of invalid type.");
        return;
    }
#endif

    setUpPublishers(nodeHandle);
}

bool IcpMatchesPublisher::isLatched() const
{
#ifndef ROS2_BUILD
    if (!parameters_.publishSurfaceNormals_)
    {
        return readingPointsMatchedPointCloudPublisher_->isLatched() || referencePointsMatchedPointCloudPublisher_->isLatched();
    }

    return readingPointsMatchedPointCloudPublisher_->isLatched() || readingPointsMatchedNormalMarkersPublisher_->isLatched()
        || referencePointsMatchedPointCloudPublisher_->isLatched() || referencePointsMatchedNormalMarkersPublisher_->isLatched();
#else
    return readingPointsMatchedPointCloudPublisher_->get_actual_qos().durability() == rclcpp::DurabilityPolicy::TransientLocal
        || referencePointsMatchedPointCloudPublisher_->get_actual_qos().durability() == rclcpp::DurabilityPolicy::TransientLocal
        || (parameters_.publishSurfaceNormals_
            && (readingPointsMatchedNormalMarkersPublisher_->get_actual_qos().durability() == rclcpp::DurabilityPolicy::TransientLocal
                || referencePointsMatchedNormalMarkersPublisher_->get_actual_qos().durability()
                    == rclcpp::DurabilityPolicy::TransientLocal));
#endif
}

size_t IcpMatchesPublisher::getNumSubscribers() const
{
    if (!parameters_.publishSurfaceNormals_)
    {
#ifndef ROS2_BUILD
        return std::max(readingPointsMatchedPointCloudPublisher_->getNumSubscribers(),
                        referencePointsMatchedPointCloudPublisher_->getNumSubscribers());
#else
        return std::max(readingPointsMatchedPointCloudPublisher_->get_subscription_count(),
                        referencePointsMatchedPointCloudPublisher_->get_subscription_count());
#endif
    }

#ifndef ROS2_BUILD
    return std::max(std::max(readingPointsMatchedPointCloudPublisher_->getNumSubscribers(),
                             readingPointsMatchedNormalMarkersPublisher_->getNumSubscribers()),
                    std::max(referencePointsMatchedPointCloudPublisher_->getNumSubscribers(),
                             referencePointsMatchedNormalMarkersPublisher_->getNumSubscribers()));
#else
    return std::max(std::max(readingPointsMatchedPointCloudPublisher_->get_subscription_count(),
                             readingPointsMatchedNormalMarkersPublisher_->get_subscription_count()),
                    std::max(referencePointsMatchedPointCloudPublisher_->get_subscription_count(),
                             referencePointsMatchedNormalMarkersPublisher_->get_subscription_count()));
#endif
}

void IcpMatchesPublisher::processMatches(const StampedPointCloud& readingPointCloud,
                                         const StampedPointCloud& referencePointCloud,
                                         const PmMatches& matches,
                                         const PmOutlierWeights& outlierWeights)
{
    if (getNumSubscribers() == 0u && !isLatched())
    {
#ifndef ROS2_BUILD
        ROS_DEBUG("Skipping extraction of matched points for visualization. No subscribers detected for topic '%s'",
                  parameters_.readingPointCloudPublisherTopic_.c_str());
#else
        RCLCPP_DEBUG(rclcpp::get_logger("PointmatcherRos"),
                     "Skipping extraction of matched points for visualization. No subscribers detected for topic '%s'",
                     parameters_.readingPointCloudPublisherTopic_.c_str());
#endif
        return;
    }

    // Build variables for direct access to data points member.
    const auto& readingPoints = readingPointCloud.dataPoints_;
    const auto& referencePoints = referencePointCloud.dataPoints_;
    const PmIndex numberOfReadingPoints{ readingPoints.features.cols() };
    const PmIndex numberOfMatchedPoints{ (outlierWeights.array() != 0.0).count() };

    // Initialize containers for matched points.
    auto& readingPointsMatched = readingPointsMatchedPointCloud_.dataPoints_;
    auto& referencePointsMatched = referencePointsMatchedPointCloud_.dataPoints_;
    readingPointsMatched = readingPoints.createSimilarEmpty(numberOfMatchedPoints);
    referencePointsMatched = referencePoints.createSimilarEmpty(numberOfMatchedPoints);

    // Count the number of nearest neighbors computed per point.
    const PmIndex knn{ outlierWeights.rows() };

    PmIndex j = 0;
    for (PmIndex i = 0; i < numberOfReadingPoints; ++i) // nb pts
    {
        for (PmIndex k = 0; k < knn; k++) // knn
        {
            if (matches.dists(k, i) == PmMatches::InvalidDist)
            {
                continue;
            }

            if (outlierWeights(k, i) == 0.0)
            {
                continue;
            }

            readingPointsMatched.features.col(j) = readingPoints.features.col(i);
            readingPointsMatched.descriptors.col(j) = readingPoints.descriptors.col(i);

            const PmIndex refIndex(matches.ids(k, i));
            referencePointsMatched.features.col(j) = referencePoints.features.col(refIndex);
            referencePointsMatched.descriptors.col(j) = referencePoints.descriptors.col(refIndex);
            ++j;
        }
    }

    readingPointsMatchedPointCloud_.header_ = readingPointCloud.header_;
    referencePointsMatchedPointCloud_.header_ = readingPointCloud.header_;
}

void IcpMatchesPublisher::publish() const
{
#ifndef ROS2_BUILD
    if (readingPointsMatchedPointCloudPublisher_ == std::nullopt || referencePointsMatchedPointCloudPublisher_ == std::nullopt)
    {
        ROS_ERROR("Point cloud data cannot be published: Publisher '%s' has not been initialized properly.",
                  parameters_.readingPointCloudPublisherTopic_.c_str());
#else
    if (!readingPointsMatchedPointCloudPublisher_ || !referencePointsMatchedPointCloudPublisher_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("PointmatcherRos"),
                     "Point cloud data cannot be published: Publisher '%s' has not been initialized properly.",
                     parameters_.readingPointCloudPublisherTopic_.c_str());
#endif
        return;
    }

    publishPointClouds();

    if (parameters_.publishSurfaceNormals_)
    {
        publishSurfaceNormals();
    }
}

#ifndef ROS2_BUILD
void IcpMatchesPublisher::setUpPublishers(ros::NodeHandle nodeHandle)
#else
void IcpMatchesPublisher::setUpPublishers(rclcpp::Node::SharedPtr const& nodeHandle)
#endif
{
#ifndef ROS2_BUILD
    readingPointsMatchedPointCloudPublisher_ = nodeHandle.advertise<sensor_msgs::PointCloud2>(
        parameters_.readingPointCloudPublisherTopic_, parameters_.queueSizeOfPublishers_, parameters_.latchPublishers_);

    referencePointsMatchedPointCloudPublisher_ = nodeHandle.advertise<sensor_msgs::PointCloud2>(
        parameters_.referencePointCloudPublisherTopic_, parameters_.queueSizeOfPublishers_, parameters_.latchPublishers_);
#else
    rclcpp::QoS qos(parameters_.queueSizeOfPublishers_);
    if (parameters_.latchPublishers_)
    {
        qos.transient_local();
    }

    readingPointsMatchedPointCloudPublisher_ =
        nodeHandle->create_publisher<sensor_msgs::msg::PointCloud2>(parameters_.readingPointCloudPublisherTopic_, qos);
    referencePointsMatchedPointCloudPublisher_ =
        nodeHandle->create_publisher<sensor_msgs::msg::PointCloud2>(parameters_.referencePointCloudPublisherTopic_, qos);
#endif

    if (parameters_.publishSurfaceNormals_)
    {
#ifndef ROS2_BUILD
        readingPointsMatchedNormalMarkersPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>(
            parameters_.readingPointCloudPublisherTopic_ + "_" + parameters_.normals_.pointCloudFieldId_,
            parameters_.queueSizeOfPublishers_,
            parameters_.latchPublishers_);
        referencePointsMatchedNormalMarkersPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>(
            parameters_.referencePointCloudPublisherTopic_ + "_" + parameters_.normals_.pointCloudFieldId_,
            parameters_.queueSizeOfPublishers_,
            parameters_.latchPublishers_);
#else
        readingPointsMatchedNormalMarkersPublisher_ = nodeHandle->create_publisher<visualization_msgs::msg::Marker>(
            parameters_.readingPointCloudPublisherTopic_ + "_" + parameters_.normals_.pointCloudFieldId_, qos);
        referencePointsMatchedNormalMarkersPublisher_ = nodeHandle->create_publisher<visualization_msgs::msg::Marker>(
            parameters_.referencePointCloudPublisherTopic_ + "_" + parameters_.normals_.pointCloudFieldId_, qos);
#endif
    }
}

void IcpMatchesPublisher::publishPointClouds() const
{
#ifndef ROS2_BUILD
    if (readingPointsMatchedPointCloudPublisher_->getNumSubscribers() > 0u || readingPointsMatchedPointCloudPublisher_->isLatched())
    {
        ROS_DEBUG("Publishing point cloud for publisher '%s'.", parameters_.readingPointCloudPublisherTopic_.c_str());
#else
    if (readingPointsMatchedPointCloudPublisher_->get_subscription_count() != 0
        || readingPointsMatchedPointCloudPublisher_->get_actual_qos().durability() == rclcpp::DurabilityPolicy::TransientLocal)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("PointmatcherRos"),
                     "Publishing point cloud for publisher '%s'.",
                     parameters_.readingPointCloudPublisherTopic_.c_str());
#endif
        readingPointsMatchedPointCloudPublisher_->publish(readingPointsMatchedPointCloud_.toRosMsg());
    }
#ifndef ROS2_BUILD
    if (referencePointsMatchedPointCloudPublisher_->getNumSubscribers() > 0u || referencePointsMatchedPointCloudPublisher_->isLatched())
    {
        ROS_DEBUG("Publishing point cloud for publisher '%s'.", parameters_.referencePointCloudPublisherTopic_.c_str());
#else
    if (referencePointsMatchedPointCloudPublisher_->get_subscription_count() != 0
        || referencePointsMatchedPointCloudPublisher_->get_actual_qos().durability() == rclcpp::DurabilityPolicy::TransientLocal)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("PointmatcherRos"),
                     "Publishing point cloud for publisher '%s'.",
                     parameters_.referencePointCloudPublisherTopic_.c_str());
#endif
        referencePointsMatchedPointCloudPublisher_->publish(referencePointsMatchedPointCloud_.toRosMsg());
    }
}

void IcpMatchesPublisher::publishSurfaceNormals() const
{
#ifndef ROS2_BUILD
    if (readingPointsMatchedNormalMarkersPublisher_->getNumSubscribers() > 0u || readingPointsMatchedNormalMarkersPublisher_->isLatched())
    {
        auto readingPointsMatchedNormalMarkers{ generateMarkersForSurfaceNormalVectors(readingPointsMatchedPointCloud_,
                                                                                       readingPointsMatchedPointCloud_.header_.stamp,
                                                                                       parameters_.normals_,
                                                                                       colorMap_[parameters_.readingMarkersColor_]) };
        if (readingPointsMatchedNormalMarkers != std::nullopt)
        {
            ROS_DEBUG("Publishing point cloud surface normals for publisher '%s'.", parameters_.readingPointCloudPublisherTopic_.c_str());
#else
    if (readingPointsMatchedNormalMarkersPublisher_->get_subscription_count() != 0
        || readingPointsMatchedNormalMarkersPublisher_->get_actual_qos().durability() == rclcpp::DurabilityPolicy::TransientLocal)
    {
        auto readingPointsMatchedNormalMarkers = generateMarkersForSurfaceNormalVectors(readingPointsMatchedPointCloud_,
                                                                                        readingPointsMatchedPointCloud_.header_.stamp,
                                                                                        parameters_.normals_,
                                                                                        colorMap_[parameters_.readingMarkersColor_]);

        if (readingPointsMatchedNormalMarkers.has_value())
        {
            RCLCPP_DEBUG(rclcpp::get_logger("PointmatcherRos"),
                         "Publishing point cloud surface normals for publisher '%s'.",
                         parameters_.readingPointCloudPublisherTopic_.c_str());
#endif
            readingPointsMatchedNormalMarkersPublisher_->publish(readingPointsMatchedNormalMarkers.value());
        }
    }

#ifndef ROS2_BUILD
    if (referencePointsMatchedNormalMarkersPublisher_->getNumSubscribers() > 0u
        || referencePointsMatchedNormalMarkersPublisher_->isLatched())
    {
        auto referencePointsMatchedNormalMarkers{ generateMarkersForSurfaceNormalVectors(referencePointsMatchedPointCloud_,
                                                                                         referencePointsMatchedPointCloud_.header_.stamp,
                                                                                         parameters_.normals_,
                                                                                         colorMap_[parameters_.referenceMarkersColor_]) };
        if (referencePointsMatchedNormalMarkers != std::nullopt)
        {
            ROS_DEBUG("Publishing point cloud surface normals for publisher '%s'.", parameters_.referencePointCloudPublisherTopic_.c_str());
#else
    if (referencePointsMatchedNormalMarkersPublisher_->get_subscription_count() != 0
        || referencePointsMatchedNormalMarkersPublisher_->get_actual_qos().durability() == rclcpp::DurabilityPolicy::TransientLocal)
    {
        auto referencePointsMatchedNormalMarkers = generateMarkersForSurfaceNormalVectors(referencePointsMatchedPointCloud_,
                                                                                          referencePointsMatchedPointCloud_.header_.stamp,
                                                                                          parameters_.normals_,
                                                                                          colorMap_[parameters_.referenceMarkersColor_]);
        if (referencePointsMatchedNormalMarkers.has_value())
        {
            RCLCPP_DEBUG(rclcpp::get_logger("PointmatcherRos"),
                         "Publishing point cloud surface normals for publisher '%s'.",
                         parameters_.referencePointCloudPublisherTopic_.c_str());
#endif
            referencePointsMatchedNormalMarkersPublisher_->publish(referencePointsMatchedNormalMarkers.value());
        }
    }
}


} // namespace pointmatcher_ros