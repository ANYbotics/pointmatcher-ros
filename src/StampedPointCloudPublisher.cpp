
#include "pointmatcher_ros/StampedPointCloudPublisher.h"

#ifndef ROS2_BUILD
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/PointCloud2.h>
#endif

namespace pointmatcher_ros
{

StampedPointCloudPublisher::StampedPointCloudPublisher() = default;

#ifndef ROS2_BUILD
void StampedPointCloudPublisher::advertiseFromRosParameters(ros::NodeHandle nodeHandle, const std::string& parametersKey)
{
    if (!nodeHandle.getParam("publishers/" + parametersKey + "/topic", parameters_.pointCloudPublisherTopic_))
    {
        ROS_ERROR("Topic name for point cloud publisher could not be fetched from the ROS Parameter Server.");
        return;
    }
    parameters_.queueSizeOfPublishers_ = nodeHandle.param("publishers/" + parametersKey + "/queue_size", 1);
    parameters_.latchPublishers_ = nodeHandle.param("publishers/" + parametersKey + "/latch", false);
    parameters_.publishSurfaceNormals_ = nodeHandle.param("publishers/" + parametersKey + "/surface_normals", false);
    parameters_.markersColor_ = static_cast<ColorKey>(
        nodeHandle.param("publishers/" + parametersKey + "/surface_normals_color", static_cast<int>(ColorKey::kWhite)));
#else
void StampedPointCloudPublisher::advertiseFromRosParameters(rclcpp::Node::SharedPtr const& nodeHandle, std::string const& parametersKey)
{
    bool success;
    try
    {
        success = nodeHandle->get_parameter<std::string>("publishers." + parametersKey + ".topic", parameters_.pointCloudPublisherTopic_);
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
        parameters_.queueSizeOfPublishers_ = nodeHandle->get_parameter_or<int>("publishers." + parametersKey + ".queue_size", 1);
        parameters_.latchPublishers_ = nodeHandle->get_parameter_or<bool>("publishers." + parametersKey + ".latch", false);
        parameters_.publishSurfaceNormals_ = nodeHandle->get_parameter_or<bool>("publishers." + parametersKey + ".surface_normals", false);
        parameters_.markersColor_ = static_cast<ColorKey>(nodeHandle->get_parameter_or<int>(
            "publishers." + parametersKey + ".surface_normals_color", static_cast<int>(ColorKey::kWhite)));
    }
    catch (rclcpp::exceptions::InvalidParameterTypeException const&)
    {
        RCLCPP_ERROR(rclcpp::get_logger("PointmatcherRos"), "Some of the parameters have values of invalid type.");
        return;
    }
#endif

    setUpPublishers(nodeHandle);
}

#ifndef ROS2_BUILD
void StampedPointCloudPublisher::advertiseFromTopicName(ros::NodeHandle nodeHandle, const std::string& topicName, const int queueSize,
                                                        const bool latch, const bool publishSurfaceNormals, const ColorKey color)
#else
void StampedPointCloudPublisher::advertiseFromTopicName(rclcpp::Node::SharedPtr const& nodeHandle, std::string const& topicName,
                                                        int queueSize, bool latch, bool publishSurfaceNormals, ColorKey color)
#endif
{
    parameters_.pointCloudPublisherTopic_ = topicName;
    parameters_.queueSizeOfPublishers_ = queueSize;
    parameters_.latchPublishers_ = latch;
    parameters_.publishSurfaceNormals_ = publishSurfaceNormals;
    parameters_.markersColor_ = color;

    setUpPublishers(nodeHandle);
}

bool StampedPointCloudPublisher::isLatched() const
{
#ifndef ROS2_BUILD
    if (!parameters_.publishSurfaceNormals_)
    {
        return pointCloudPublisher_->isLatched();
    }

    return pointCloudPublisher_->isLatched() || normalsMarkersPublisher_->isLatched();
#else
    return pointCloudPublisher_->get_actual_qos().durability() == rclcpp::DurabilityPolicy::TransientLocal
        || (parameters_.publishSurfaceNormals_
            && normalsMarkersPublisher_->get_actual_qos().durability() == rclcpp::DurabilityPolicy::TransientLocal);
#endif
}

size_t StampedPointCloudPublisher::getNumSubscribers() const
{
#ifndef ROS2_BUILD
    if (!parameters_.publishSurfaceNormals_)
    {
        return pointCloudPublisher_->getNumSubscribers();
    }

    return std::max(pointCloudPublisher_->getNumSubscribers(), normalsMarkersPublisher_->getNumSubscribers());
#else
    auto const count_cloud_publisher = pointCloudPublisher_->get_subscription_count();
    return !parameters_.publishSurfaceNormals_ ? count_cloud_publisher
                                               : std::max(count_cloud_publisher, normalsMarkersPublisher_->get_subscription_count());
#endif
}

#ifndef ROS2_BUILD
void StampedPointCloudPublisher::publish(const StampedPointCloud& pointCloud, const ros::Time targetStamp) const
{
    if (pointCloudPublisher_ == std::nullopt)
    {
        ROS_ERROR("Point cloud data cannot be published: Publisher '%s' has not been initialized properly.",
                  parameters_.pointCloudPublisherTopic_.c_str());
#else
void StampedPointCloudPublisher::publish(StampedPointCloud const& pointCloud, rclcpp::Time const& targetStamp) const
{
    if (!pointCloudPublisher_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("PointmatcherRos"),
                     "Point cloud data cannot be published: Publisher '%s' has not been initialized properly.",
                     parameters_.pointCloudPublisherTopic_.c_str());
#endif
        return;
    }

    // Overwrite timestamp if a specific one is provided.
#ifndef ROS2_BUILD
    ros::Time timestamp{ pointCloud.header_.stamp };
    if (targetStamp != ros::Time(0))
#else
    rclcpp::Time timestamp(pointCloud.header_.stamp);
    if (targetStamp.nanoseconds() != 0)
#endif
    {
        timestamp = targetStamp;
    }

    publishPointCloud(pointCloud, timestamp);

    if (parameters_.publishSurfaceNormals_)
    {
        publishSurfaceNormals(pointCloud, timestamp);
    }
}

#ifndef ROS2_BUILD
void StampedPointCloudPublisher::setUpPublishers(ros::NodeHandle nodeHandle)
{
    pointCloudPublisher_ = nodeHandle.advertise<sensor_msgs::PointCloud2>(
        parameters_.pointCloudPublisherTopic_, parameters_.queueSizeOfPublishers_, parameters_.latchPublishers_);

    if (parameters_.publishSurfaceNormals_)
    {
        normalsMarkersPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>(parameters_.pointCloudPublisherTopic_ + "_"
                                                                                        + parameters_.normals_.pointCloudFieldId_,
                                                                                    parameters_.queueSizeOfPublishers_,
                                                                                    parameters_.latchPublishers_);
    }
}
#else
void StampedPointCloudPublisher::setUpPublishers(rclcpp::Node::SharedPtr const& nodeHandle)
{
    rclcpp::QoS qos(parameters_.queueSizeOfPublishers_);
    if (parameters_.latchPublishers_)
    {
        qos.transient_local();
    }

    pointCloudPublisher_ = nodeHandle->create_publisher<sensor_msgs::msg::PointCloud2>(parameters_.pointCloudPublisherTopic_, qos);

    if (parameters_.publishSurfaceNormals_)
    {
        normalsMarkersPublisher_ = nodeHandle->create_publisher<visualization_msgs::msg::Marker>(
            parameters_.pointCloudPublisherTopic_ + "_" + parameters_.normals_.pointCloudFieldId_, qos);
    }
}
#endif

#ifndef ROS2_BUILD
void StampedPointCloudPublisher::publishPointCloud(const StampedPointCloud& pointCloud, const ros::Time& timestamp) const
{
    if (pointCloudPublisher_->getNumSubscribers() == 0u && !pointCloudPublisher_->isLatched())
    {
        return;
    }
    ROS_DEBUG("Publishing point cloud for publisher '%s'.", parameters_.pointCloudPublisherTopic_.c_str());

    pointCloudPublisher_->publish(pointCloud.toRosMsg(timestamp));
}
#else
void StampedPointCloudPublisher::publishPointCloud(StampedPointCloud const& pointCloud, rclcpp::Time const& timestamp) const
{
    if (pointCloudPublisher_->get_subscription_count() != 0
        || pointCloudPublisher_->get_actual_qos().durability() == rclcpp::DurabilityPolicy::TransientLocal)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("PointmatcherRos"),
                     "Publishing point cloud for publisher '%s'.",
                     parameters_.pointCloudPublisherTopic_.c_str());

        pointCloudPublisher_->publish(pointCloud.toRosMsg(timestamp));
    }
}
#endif

#ifndef ROS2_BUILD
void StampedPointCloudPublisher::publishSurfaceNormals(const StampedPointCloud& pointCloud, const ros::Time& timestamp) const
{
    if (normalsMarkersPublisher_->getNumSubscribers() > 0u || normalsMarkersPublisher_->isLatched())
    {
        auto normalMarkers{ generateMarkersForSurfaceNormalVectors(
            pointCloud, timestamp, parameters_.normals_, colorMap_[parameters_.markersColor_]) };
        if (normalMarkers != std::nullopt)
        {
            ROS_DEBUG("Publishing point cloud surface normals for publisher '%s'.", parameters_.pointCloudPublisherTopic_.c_str());
#else
void StampedPointCloudPublisher::publishSurfaceNormals(StampedPointCloud const& pointCloud, rclcpp::Time const& timestamp) const
{
    if (normalsMarkersPublisher_->get_subscription_count() != 0
        || pointCloudPublisher_->get_actual_qos().durability() == rclcpp::DurabilityPolicy::TransientLocal)
    {
        auto normalMarkers =
            generateMarkersForSurfaceNormalVectors(pointCloud, timestamp, parameters_.normals_, colorMap_[parameters_.markersColor_]);
        if (normalMarkers.has_value())
        {
            RCLCPP_DEBUG(rclcpp::get_logger("PointmatcherRos"),
                         "Publishing point cloud surface normals for publisher '%s'.",
                         parameters_.pointCloudPublisherTopic_.c_str());
#endif
            normalsMarkersPublisher_->publish(normalMarkers.value());
        }
    }
}


} // namespace pointmatcher_ros
