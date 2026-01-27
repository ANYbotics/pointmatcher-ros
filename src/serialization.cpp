#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG

#include "pointmatcher_ros/serialization.h"

namespace pointmatcher_ros
{
template<typename T>
sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg(const typename PointMatcher<T>::DataPoints& pmCloud, const std::string& frame_id,
                                                   const ros::Time& stamp)
{
    sensor_msgs::PointCloud2 rosCloud;
    typedef sensor_msgs::PointField PF;

    // check type and get sizes
    uint8_t dataType;
    size_t scalarSize;
    if (typeid(T) == typeid(float))
    {
        dataType = PF::FLOAT32;
        scalarSize = 4;
    }
    else
    {
        dataType = PF::FLOAT64;
        scalarSize = 8;
    }

    // build labels

    // features
    unsigned offset(0);
    assert(!pmCloud.featureLabels.empty());
    assert(pmCloud.featureLabels[pmCloud.featureLabels.size() - 1].text == "pad");
    for (auto it(pmCloud.featureLabels.begin()); it != pmCloud.featureLabels.end(); ++it)
    {
        // last label is padding
        if ((it + 1) == pmCloud.featureLabels.end())
            break;
        PF pointField;
        pointField.name = it->text;
        pointField.offset = offset;
        pointField.datatype = dataType;
        pointField.count = it->span;
        rosCloud.fields.push_back(pointField);
        offset += it->span * scalarSize;
    }
    bool addZ(false);
    if (!pmCloud.featureLabels.contains("z"))
    {
        PF pointField;
        pointField.name = "z";
        pointField.offset = offset;
        pointField.datatype = dataType;
        pointField.count = 1;
        rosCloud.fields.push_back(pointField);
        offset += scalarSize;
        addZ = true;
    }

    // descriptors
    const bool isDescriptor(!pmCloud.descriptorLabels.empty());
    bool hasColor(false);
    unsigned colorPos(0);
    unsigned colorCount(0);
    unsigned inDescriptorPos(0);
    for (auto it(pmCloud.descriptorLabels.begin()); it != pmCloud.descriptorLabels.end(); ++it)
    {
        PF pointField;
        if (it->text == "normals")
        {
            assert((it->span == 2) || (it->span == 3));
            pointField.datatype = dataType;
            pointField.name = "normal_x";
            pointField.offset = offset;
            pointField.count = 1;
            rosCloud.fields.push_back(pointField);
            offset += scalarSize;
            pointField.name = "normal_y";
            pointField.offset = offset;
            pointField.count = 1;
            rosCloud.fields.push_back(pointField);
            offset += scalarSize;
            if (it->span == 3)
            {
                pointField.name = "normal_z";
                pointField.offset = offset;
                pointField.count = 1;
                rosCloud.fields.push_back(pointField);
                offset += scalarSize;
            }
        }
        else if (it->text == "color")
        {
            colorPos = inDescriptorPos;
            colorCount = it->span;
            hasColor = true;
            pointField.datatype = (colorCount == 4) ? uint8_t(PF::UINT32) : uint8_t(PF::FLOAT32);
            pointField.name = (colorCount == 4) ? "rgba" : "rgb";
            pointField.offset = offset;
            pointField.count = 1;
            rosCloud.fields.push_back(pointField);
            offset += 4;
        }
        else
        {
            pointField.datatype = dataType;
            pointField.name = it->text;
            pointField.offset = offset;
            pointField.count = it->span;
            rosCloud.fields.push_back(pointField);
            offset += it->span * scalarSize;
        }
        inDescriptorPos += it->span;
    }

    // fill cloud with data
    rosCloud.header.frame_id = frame_id;
    rosCloud.header.stamp = stamp;
    rosCloud.height = 1;
    rosCloud.width = pmCloud.features.cols();
    rosCloud.is_bigendian = false;
    rosCloud.point_step = offset;
    rosCloud.row_step = rosCloud.point_step * rosCloud.width;
    rosCloud.is_dense = true;
    rosCloud.data.resize(rosCloud.row_step * rosCloud.height);

    const unsigned featureDim(pmCloud.features.rows() - 1);
    const unsigned descriptorDim(pmCloud.descriptors.rows());

    assert(descriptorDim == inDescriptorPos);
    const unsigned postColorPos(colorPos + colorCount);
    assert(postColorPos <= inDescriptorPos);
    const unsigned postColorCount(descriptorDim - postColorPos);

    for (unsigned pt(0); pt < rosCloud.width; ++pt)
    {
        uint8_t* fPtr(&rosCloud.data[pt * offset]);

        memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.features(0, pt)), scalarSize * featureDim);
        fPtr += scalarSize * featureDim;
        if (addZ)
        {
            memset(fPtr, 0, scalarSize);
            fPtr += scalarSize;
        }
        if (isDescriptor)
        {
            if (hasColor)
            {
                // before color
                memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.descriptors(0, pt)), scalarSize * colorPos);
                fPtr += scalarSize * colorPos;
                // compact color
                uint32_t rgba;
                unsigned colorR(unsigned(pmCloud.descriptors(colorPos + 0, pt) * 255.) & 0xFF);
                unsigned colorG(unsigned(pmCloud.descriptors(colorPos + 1, pt) * 255.) & 0xFF);
                unsigned colorB(unsigned(pmCloud.descriptors(colorPos + 2, pt) * 255.) & 0xFF);
                unsigned colorA(0);
                if (colorCount == 4)
                    colorA = unsigned(pmCloud.descriptors(colorPos + 3, pt) * 255.) & 0xFF;
                rgba = colorA << 24 | colorR << 16 | colorG << 8 | colorB;
                memcpy(fPtr, reinterpret_cast<const uint8_t*>(&rgba), 4);
                fPtr += 4;
                // after color
                if (postColorCount > 0)
                {
                    memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.descriptors(postColorPos, pt)), scalarSize * postColorCount);
                }
                fPtr += scalarSize * postColorCount;
            }
            else
            {
                memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.descriptors(0, pt)), scalarSize * descriptorDim);
                fPtr += scalarSize * descriptorDim;
            }
        }
    }

    // fill remaining information
    rosCloud.header.frame_id = frame_id;
    rosCloud.header.stamp = stamp;

    return rosCloud;
}

template sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg<float>(const PointMatcher<float>::DataPoints& pmCloud,
                                                                   const std::string& frame_id, const ros::Time& stamp);
template sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg<double>(const PointMatcher<double>::DataPoints& pmCloud,
                                                                    const std::string& frame_id, const ros::Time& stamp);
} // namespace pointmatcher_ros