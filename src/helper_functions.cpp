
#include "pointmatcher_ros/helper_functions.h"

#ifndef ROS2_BUILD
#include <eigen3/Eigen/src/Geometry/Transform.h>
#else
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#endif

namespace pointmatcher_ros
{

#ifndef ROS2_BUILD
PmTf convertRosTfMsgToPmTf(const geometry_msgs::TransformStamped& tfMsg)
{
    tf::StampedTransform tf;
    tf::transformStampedMsgToTF(tfMsg, tf);
    return pointmatcher_ros::convertRosTfToPmTf(tf);
}
#else
PmTf convertRosTfMsgToPmTf(geometry_msgs::msg::TransformStamped const& tfMsg)
{
    PmTf pmTf;
    pmTf.sourceFrameId_ = tfMsg.child_frame_id;
    pmTf.targetFrameId_ = tfMsg.header.frame_id;
    pmTf.stamp_ = tfMsg.header.stamp;

    Eigen::Isometry3d const& isometry3d = tf2::transformToEigen(tfMsg);
    pmTf.parameters_ = isometry3d.matrix().cast<float>();

    return pmTf;
}
#endif

#ifndef ROS2_BUILD
geometry_msgs::TransformStamped convertPmTfToTfMsg(const PmTf& pmTf)
{
    geometry_msgs::TransformStamped tfMsg;
    tf::transformStampedTFToMsg(pointmatcher_ros::convertPmTfToTf(pmTf), tfMsg);
    return tfMsg;
}
#else
geometry_msgs::msg::TransformStamped convertPmTfToTfMsg(PmTf const& pmTf)
{
    Eigen::Affine3d affine3d;
    affine3d.matrix() = pmTf.parameters_.cast<double>();

    geometry_msgs::msg::TransformStamped tfMsg = tf2::eigenToTransform(affine3d);

    tfMsg.header.stamp = pmTf.stamp_;
    tfMsg.header.frame_id = pmTf.targetFrameId_;
    tfMsg.child_frame_id = pmTf.sourceFrameId_;

    return tfMsg;
}
#endif

#ifndef ROS2_BUILD
PmTf convertRosTfToPmTf(const tf::StampedTransform& tf)
{
    PmTf pmTf;
    pmTf.sourceFrameId_ = tf.child_frame_id_;
    pmTf.targetFrameId_ = tf.frame_id_;
    pmTf.stamp_ = tf.stamp_;
    Eigen::Affine3d affineTf;
    tf::transformTFToEigen(tf, affineTf);
    pmTf.parameters_ = affineTf.matrix().cast<float>();
    return pmTf;
}

tf::StampedTransform convertPmTfToTf(const PmTf& pmTf)
{
    tf::StampedTransform tf;
    tf.child_frame_id_ = pmTf.sourceFrameId_;
    tf.frame_id_ = pmTf.targetFrameId_;
    tf.stamp_ = pmTf.stamp_;
    tf::Matrix3x3 basis(pmTf.parameters_(0, 0),
                        pmTf.parameters_(0, 1),
                        pmTf.parameters_(0, 2),
                        pmTf.parameters_(1, 0),
                        pmTf.parameters_(1, 1),
                        pmTf.parameters_(1, 2),
                        pmTf.parameters_(2, 0),
                        pmTf.parameters_(2, 1),
                        pmTf.parameters_(2, 2));
    tf.setBasis(basis);
    tf::Vector3 origin(pmTf.parameters_(0, 3), pmTf.parameters_(1, 3), pmTf.parameters_(2, 3));
    tf.setOrigin(origin);
    return tf;
}

geometry_msgs::PoseStamped convertRosTfToRosTfMsg(const tf::StampedTransform& tf)
{
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = tf.frame_id_;
    poseStamped.header.seq = 0;
    poseStamped.header.stamp = tf.stamp_;
    poseStamped.pose.position.x = tf.getOrigin().getX();
    poseStamped.pose.position.y = tf.getOrigin().getY();
    poseStamped.pose.position.z = tf.getOrigin().getZ();
    poseStamped.pose.orientation.w = tf.getRotation().getW();
    poseStamped.pose.orientation.x = tf.getRotation().getX();
    poseStamped.pose.orientation.y = tf.getRotation().getY();
    poseStamped.pose.orientation.z = tf.getRotation().getZ();
    return poseStamped;
}

tf::StampedTransform convertPoseStampedMsgToRosTf(const geometry_msgs::PoseStamped& pose, const std::string& childFrameId)
{
    const tf::Vector3 origin(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    const tf::Quaternion rotation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::StampedTransform tf;
    tf.frame_id_ = pose.header.frame_id;
    tf.child_frame_id_ = childFrameId;
    tf.stamp_ = pose.header.stamp;
    tf.setOrigin(origin);
    tf.setRotation(rotation);
    return tf;
}

#endif

#ifndef ROS2_BUILD
geometry_msgs::PoseStamped convertPmTfToPose(const PmTf& pmTf)
{
    return convertRosTfToRosTfMsg(convertPmTfToTf(pmTf));
}
#else
geometry_msgs::msg::PoseStamped convertPmTfToPose(PmTf const& pmTf)
{
    geometry_msgs::msg::PoseStamped poseStamped;
    poseStamped.header.frame_id = pmTf.targetFrameId_;
    poseStamped.header.stamp = pmTf.stamp_;

    Eigen::Isometry3d isometry3d;
    isometry3d.matrix() = pmTf.parameters_.cast<double>();
    auto const& translation = isometry3d.translation();
    poseStamped.pose.position.x = translation.x();
    poseStamped.pose.position.y = translation.y();
    poseStamped.pose.position.z = translation.z();

    Eigen::Quaterniond rotation(isometry3d.rotation());
    poseStamped.pose.orientation.x = rotation.x();
    poseStamped.pose.orientation.y = rotation.y();
    poseStamped.pose.orientation.z = rotation.z();
    poseStamped.pose.orientation.w = rotation.w();

    return poseStamped;
}
#endif

#ifndef ROS2_BUILD
PmTf convertPoseStampedMsgToPmTf(const geometry_msgs::PoseStamped& pose, const std::string& childFrameId)
{
    return convertRosTfToPmTf(convertPoseStampedMsgToRosTf(pose, childFrameId));
}
#else
PmTf convertPoseStampedMsgToPmTf(geometry_msgs::msg::PoseStamped const& pose, std::string const& childFrameId)
{
    PmTf pmTf;
    pmTf.sourceFrameId_ = childFrameId;
    pmTf.targetFrameId_ = pose.header.frame_id;
    pmTf.stamp_ = pose.header.stamp;

    Eigen::Isometry3d isometry3d = Eigen::Isometry3d::Identity();
    Eigen::Vector3d translation(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    isometry3d.translate(translation);
    Eigen::Quaterniond rotation(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    rotation.normalize();
    isometry3d.rotate(rotation);

    pmTf.parameters_ = isometry3d.matrix().cast<float>();
    return pmTf;
}
#endif
} // namespace pointmatcher_ros
