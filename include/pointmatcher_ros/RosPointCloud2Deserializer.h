#pragma once

// std
#include <vector>

// pointmatcher
#include <pointmatcher/IO.h>
#include <pointmatcher/PointMatcher.h>

// sensor_msgs
#ifndef ROS2_BUILD
#include <sensor_msgs/PointCloud2.h>
#else
#include <sensor_msgs/msg/point_cloud2.hpp>
#endif

namespace pointmatcher_ros
{

template<typename ScalarType>
class RosPointCloud2Deserializer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PM = PointMatcher<ScalarType>;
    using DataPoints = typename PM::DataPoints;

    /**
     * @brief Deserializes a sensor_msgs/PointCloud2 objects into a Pointmatcher point cloud (DataPoints)
     * 
     * @param rosMsg        Input message of type sensor_msgs/PointCloud2.
     * @return DataPoints   Output point cloud.
     */
#ifndef ROS2_BUILD
    static DataPoints deserialize(const sensor_msgs::PointCloud2& rosMsg);
#else
    static DataPoints deserialize(sensor_msgs::msg::PointCloud2 const& rosMsg);
#endif

private:
    using PMIO = PointMatcherIO<ScalarType>;
    using PM_types = typename PMIO::PMPropTypes;
    using Index = typename DataPoints::Index;
    using Label = typename DataPoints::Label;
    using Labels = typename DataPoints::Labels;
    using IndexMatrix = typename PM::IndexMatrix;
    using IndexGridView = typename DataPoints::IndexGridView;
    using View = typename DataPoints::View;
    using FieldNamesList = std::vector<std::string>;

    /**
     * @brief Extract the feature and descriptor labels of a given sensor_msgs/PointCloud2 message.
     * 
     * @param rosMsg        Input message of type sensor_msgs/PointCloud2.
     * @param featLabels    Feature labels. Assumed to be empty, will be filled by this method.
     * @param descLabels    Descriptor labels. Assumed to be empty, will be filled by this method.
     */
#ifndef ROS2_BUILD
    static void extractFieldLabels(const sensor_msgs::PointCloud2& rosMsg, Labels& featLabels, Labels& descLabels);
#else
    static void extractFieldLabels(sensor_msgs::msg::PointCloud2 const& rosMsg, Labels& featLabels, Labels& descLabels);
#endif

    /**
     * @brief Fills data from a scalar descriptor into a Pointmatcher point cloud.
     * 
     * @param rosMsg        Input message of type sensor_msgs/PointCloud2.
     * @param fieldName     Name of the point cloud field that corresponds to the descriptor. 
     * @param pointCount    Number of points in the input point cloud.
     * @param view          View on the output point cloud, will be modified.
     */
#ifndef ROS2_BUILD
    static void fillScalarDataIntoView(const sensor_msgs::PointCloud2& rosMsg, const std::string& fieldName, const size_t pointCount,
                                       View& view);
#else
    static void fillScalarDataIntoView(sensor_msgs::msg::PointCloud2 const& rosMsg, std::string const& fieldName, size_t pointCount,
                                       View& view);
#endif

    /**
     * @brief Fills data from a vector feature or descriptor into a Pointmatcher point cloud.
     * 
     * @param rosMsg            Input message of type sensor_msgs/PointCloud2.
     * @param fieldName         Name of the point cloud field that corresponds to the descriptor. 
     * @param is3dPointCloud    Whether the point cloud is 3D or 2D.
     * @param pointCount        Number of points in the input point cloud.
     * @param view              View on the output point cloud, will be modified.
     */
#ifndef ROS2_BUILD
    static void fillVectorDataIntoView(const sensor_msgs::PointCloud2& rosMsg, const FieldNamesList& fieldNames, const bool is3dPointCloud,
                                       const size_t pointCount, View& view);
#else
    static void fillVectorDataIntoView(sensor_msgs::msg::PointCloud2 const& rosMsg, FieldNamesList const& fieldNames, bool is3dPointCloud,
                                       size_t pointCount, View& view);
#endif

    /**
     * @brief Fills data from a color descriptor into a Pointmatcher point cloud.
     * 
     * @param rosMsg            Input message of type sensor_msgs/PointCloud2.
     * @param fieldName         Name of the point cloud field that corresponds to the descriptor. 
     * @param pointCount        Number of points in the input point cloud.
     * @param view              View on the output point cloud, will be modified.
     */
#ifndef ROS2_BUILD
    static void fillColorDataIntoView(const sensor_msgs::PointCloud2& rosMsg, const FieldNamesList& fieldNames, const size_t pointCount,
                                      View& view);
#else
    static void fillColorDataIntoView(sensor_msgs::msg::PointCloud2 const& rosMsg, FieldNamesList const& fieldNames, size_t pointCount,
                                      View& view);
#endif

    /**
     * @brief Fills data from a Lidar Ring descriptor into a Pointmatcher point cloud.
     * @remark This method is needed because Hesai Lidars rings are usually encoded as 16-bit uints, not floating-point.
     * 
     * @param rosMsg 
     * @param fieldName 
     * @param pointCount 
     * @param view 
     */
#ifndef ROS2_BUILD
    static void fillPerPointRingDataIntoView(const sensor_msgs::PointCloud2& rosMsg, const std::string& fieldName, const size_t pointCount,
                                             View& view);
#else
    static void fillPerPointRingDataIntoView(sensor_msgs::msg::PointCloud2 const& rosMsg, std::string const& fieldName, size_t pointCount,
                                             View& view);
#endif

    /**
     * @brief Fills data from an Absolute Timestamp descriptor into a Pointmatcher point cloud.
     * @remark This method is needed because Hesai Lidars store their timestamps in Unix Time, which breaks downstream nodes that expect a relative stamp.
     * This method transforms the timestamp to be relative to the start of the Lidar spin.
     * 
     * @param rosMsg 
     * @param fieldName 
     * @param pointCount 
     * @param view 
     */
#ifndef ROS2_BUILD
    static void fillPerPointAbsoluteTimestampDataIntoView(const sensor_msgs::PointCloud2& rosMsg, const std::string& fieldName,
                                                          const size_t pointCount, View& view);
#else
    static void fillPerPointAbsoluteTimestampDataIntoView(sensor_msgs::msg::PointCloud2 const& rosMsg, std::string const& fieldName,
                                                          size_t pointCount, View& view);
#endif

    /**
     * @brief Fills the 'index grid' from a ROS message into a Pointmatcher point cloud.
     * @remark This method expectes the ROS message to contain a non-dense (organized) point cloud.
     * 
     * @param rosMsg            Input message of type sensor_msgs/PointCloud2.
     * @param pointCount        Number of points in the input point cloud.
     * @param cloud             Point cloud. Its index grid will be modified
     */
#ifndef ROS2_BUILD
    static void fillIndexGrid(const sensor_msgs::PointCloud2& rosMsg, const size_t pointCount, DataPoints& cloud);
#else
    static void fillIndexGrid(sensor_msgs::msg::PointCloud2 const& rosMsg, size_t pointCount, DataPoints& cloud);
#endif

    /**
     * @brief Fills a Pointmatcher point cloud with data from a sensor_msgs/PointCloud2 message.
     * 
     * @param rosMsg            Input message of type sensor_msgs/PointCloud2.
     * @param is3dPointCloud    Whether the point cloud is 3D or 2D.
     * @param pointCloud        View on the output point cloud, will be modified.
     */
#ifndef ROS2_BUILD
    static void fillPointCloudValues(const sensor_msgs::PointCloud2& rosMsg, const bool is3dPointCloud, DataPoints& pointCloud);
#else
    static void fillPointCloudValues(sensor_msgs::msg::PointCloud2 const& rosMsg, bool is3dPointCloud, DataPoints& pointCloud);
#endif
};

} // namespace pointmatcher_ros