
#include "pointmatcher_ros/RosPointCloud2Deserializer.h"

// sensor_msgs
#include <sensor_msgs/point_cloud2_iterator.h>

namespace pointmatcher_ros
{

template<typename ScalarType>
void RosPointCloud2Deserializer<ScalarType>::extractFieldLabels(const sensor_msgs::PointCloud2& rosMsg,
                                                                Labels& featLabels,
                                                                Labels& descLabels)
{
    // Conversions of descriptor fields from pcl.
    // see http://www.ros.org/wiki/pcl/Overview
    std::vector<PM_types> fieldTypes;
    std::vector<bool> isFeature;
    for (auto it(rosMsg.fields.begin()); it != rosMsg.fields.end(); ++it)
    {
        const std::string& name(it->name);
        const size_t count{ std::max<size_t>(it->count, 1) };
        if (name == "x" || name == "y" || name == "z")
        {
            featLabels.push_back(Label(name, count));
            isFeature.push_back(true);
            fieldTypes.push_back(PM_types::FEATURE);
        }
        else if (name == "rgb" || name == "rgba")
        {
            descLabels.push_back(Label("color", (name == "rgba") ? 4 : 3));
            isFeature.push_back(false);
            fieldTypes.push_back(PM_types::DESCRIPTOR);
        }
        else if ((it + 1) != rosMsg.fields.end() && it->name == "normal_x" && (it + 1)->name == "normal_y")
        {
            if ((it + 2) != rosMsg.fields.end() && (it + 2)->name == "normal_z")
            {
                descLabels.push_back(Label("normals", 3));
                isFeature.push_back(false);
                isFeature.push_back(false);
                fieldTypes.push_back(PM_types::DESCRIPTOR);
                fieldTypes.push_back(PM_types::DESCRIPTOR);
                it += 2;
            }
            else
            {
                descLabels.push_back(Label("normals", 2));
                isFeature.push_back(false);
                fieldTypes.push_back(PM_types::DESCRIPTOR);
                it += 1;
            }
            isFeature.push_back(false);
            fieldTypes.push_back(PM_types::DESCRIPTOR);
        }
        else
        {
            descLabels.push_back(Label(name, count));
            isFeature.push_back(false);
            fieldTypes.push_back(PM_types::DESCRIPTOR);
        }
    }

    // Add padding after features.
    // Libpointmatcher represents points in homogeneous coordinates and padding is just another name for the scale factor.
    featLabels.push_back(Label("pad", 1));

    // TODO(ynava) The variables 'isFeature' and 'fieldTypes' are just kept for running assertions. Consider removing them and performing a runtime check.
    assert(isFeature.size() == rosMsg.fields.size());
    assert(fieldTypes.size() == rosMsg.fields.size());
}

template<typename ScalarType>
void RosPointCloud2Deserializer<ScalarType>::fillScalarDataIntoView(const sensor_msgs::PointCloud2& rosMsg, const std::string& fieldName,
                                                                    const size_t pointCount, View& view)
{
    // Use iterator to read data and write it into view.
    sensor_msgs::PointCloud2ConstIterator<ScalarType> iter(rosMsg, fieldName);
    for (size_t i = 0; i < pointCount; ++i, ++iter)
    {
        view(0, i) = *iter;
    }
}

template<typename ScalarType>
void RosPointCloud2Deserializer<ScalarType>::fillVectorDataIntoView(const sensor_msgs::PointCloud2& rosMsg,
                                                                    const std::vector<std::string>& fieldNames, const bool is3dPointCloud,
                                                                    const size_t pointCount, View& view)
{
    // Create iterators to read data from the message buffer.
    sensor_msgs::PointCloud2ConstIterator<ScalarType> iterX(rosMsg, fieldNames[0]);
    sensor_msgs::PointCloud2ConstIterator<ScalarType> iterY(rosMsg, fieldNames[1]);

    // Dispatch a deserialization routine based on dimensions.
    if (is3dPointCloud)
    {
        sensor_msgs::PointCloud2ConstIterator<ScalarType> iterZ(rosMsg, fieldNames[2]);
        for (size_t i = 0; i < pointCount; ++i, ++iterX, ++iterY, ++iterZ)
        {
            view(0, i) = *iterX;
            view(1, i) = *iterY;
            view(2, i) = *iterZ;
        }
    }
    else
    {
        for (size_t i = 0; i < pointCount; ++i, ++iterX, ++iterY)
        {
            view(0, i) = *iterX;
            view(1, i) = *iterY;
        }
    }
}

template<typename ScalarType>
void RosPointCloud2Deserializer<ScalarType>::fillPointCloudValues(const sensor_msgs::PointCloud2& rosMsg, const bool is3dPointCloud,
                                                                  DataPoints& pointCloud)
{
    const size_t pointCount{ rosMsg.width * rosMsg.height };

    // Point coordinates.
    {
        View view(pointCloud.features.block(0, 0, pointCloud.features.rows(), pointCloud.features.cols()));
        const std::vector<std::string> fieldNames{ "x", "y", "z" };
        fillVectorDataIntoView(rosMsg, fieldNames, is3dPointCloud, pointCount, view);
        pointCloud.getFeatureViewByName("pad").setOnes();
    }

    // Normals.
    if (pointCloud.descriptorExists("normals"))
    {
        View view(pointCloud.getDescriptorViewByName("normals"));
        const std::vector<std::string> fieldNames{ "normal_x", "normal_y", "normal_z" };
        fillVectorDataIntoView(rosMsg, fieldNames, is3dPointCloud, pointCount, view);
    }

    // Colors.
    if (pointCloud.descriptorExists("color"))
    {
        View view(pointCloud.getDescriptorViewByName("color"));
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iterR(rosMsg, "r");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iterG(rosMsg, "g");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iterB(rosMsg, "b");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iterA(rosMsg, "a");
        for (size_t i = 0; i < pointCount; ++i, ++iterR, ++iterG, ++iterB, ++iterA)
        {
            // PointCloud2Iterator implicitly casts to the type specified in its template arguments.
            view(0, i) = static_cast<int>(*iterR) / 255.0;
            view(1, i) = static_cast<int>(*iterG) / 255.0;
            view(2, i) = static_cast<int>(*iterB) / 255.0;
            if (view.rows() > 3)
            {
                view(3, i) = static_cast<int>(*iterA) / 255.0;
            }
        }
    }

    // Scalar descriptors.
    const std::vector<std::string> preprocessedFieldLabels{ "xyz",   "x",   "y",    "z", "normals", "normal_x", "normal_y", "normal_z",
                                                            "color", "rgb", "rgba", "r", "g",       "b",        "a" };
    for (const auto& field : rosMsg.fields)
    {
        // Ignore descriptors that we have previously written into our point cloud matrix.
        if (std::find(preprocessedFieldLabels.begin(), preprocessedFieldLabels.end(), field.name) != preprocessedFieldLabels.end())
        {
            continue;
        }

        View view{ pointCloud.getDescriptorViewByName(field.name) };
        fillScalarDataIntoView(rosMsg, field.name, pointCount, view);
    }
}

template<typename ScalarType>
typename RosPointCloud2Deserializer<ScalarType>::DataPoints RosPointCloud2Deserializer<ScalarType>::deserialize(
    const sensor_msgs::PointCloud2& rosMsg)
{
    // If the msg is empty return an empty point cloud.
    if (rosMsg.fields.empty())
    {
        return DataPoints();
    }

    // Label containers.
    Labels featLabels;
    Labels descLabels;

    // Fill field labels.
    extractFieldLabels(rosMsg, featLabels, descLabels);

    // Determine the point cloud type (2D or 3D).
    // All points are represented in homogeneous coordinates, so dim 4 -> 3D and dim 3 -> 2D.
    const bool is3dPointCloud = (featLabels.size() - 1) == 3 ? true : false;

    // Create cloud
    const size_t pointCount{ rosMsg.width * rosMsg.height };
    DataPoints pointCloud(featLabels, descLabels, pointCount);

    // Fill cloud with data.
    fillPointCloudValues(rosMsg, is3dPointCloud, pointCloud);

    return pointCloud;
}


// Explicit template instantiations for floating point types.
template class RosPointCloud2Deserializer<float>;
template class RosPointCloud2Deserializer<double>;

} // namespace pointmatcher_ros