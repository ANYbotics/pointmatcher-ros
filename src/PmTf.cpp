
#include "pointmatcher_ros/PmTf.h"

// pointmatcher_ros
#include "pointmatcher_ros/helper_functions.h"

namespace pointmatcher_ros
{

PmTf::PmTf() : transformator_(std::shared_ptr<PmTransformator>(Pm::get().REG(Transformation).create("RigidTransformation")))
{
    parameters_ = Pm::Matrix::Identity(4, 4);
}

#ifndef ROS2_BUILD
PmTf PmTf::FromRosTfMsg(const geometry_msgs::TransformStamped& tfMsg)
#else
PmTf PmTf::FromRosTfMsg(geometry_msgs::msg::TransformStamped const& tfMsg)
#endif
{
    PmTf pmTf;
    pmTf.fromRosTfMsg(tfMsg);
    return pmTf;
}

#ifndef ROS2_BUILD
void PmTf::fromRosTfMsg(const geometry_msgs::TransformStamped& tfMsg)
#else
void PmTf::fromRosTfMsg(geometry_msgs::msg::TransformStamped const& tfMsg)
#endif
{
    *this = convertRosTfMsgToPmTf(tfMsg);
}

#ifndef ROS2_BUILD
geometry_msgs::TransformStamped PmTf::toRosTfMsg() const
{
    geometry_msgs::TransformStamped tfMsg;
#else
geometry_msgs::msg::TransformStamped PmTf::toRosTfMsg() const
{
    geometry_msgs::msg::TransformStamped tfMsg;
#endif
    toRosTfMsg(tfMsg);
    return tfMsg;
}

#ifndef ROS2_BUILD
void PmTf::toRosTfMsg(geometry_msgs::TransformStamped& tfMsg) const
#else
void PmTf::toRosTfMsg(geometry_msgs::msg::TransformStamped& tfMsg) const
#endif
{
    tfMsg = convertPmTfToTfMsg(*this);
}

#ifndef ROS2_BUILD
tf::StampedTransform PmTf::toRosTf() const
{
    tf::StampedTransform tf;
    toRosTf(tf);
    return tf;
}

void PmTf::toRosTf(tf::StampedTransform& tf) const
{
    tf = convertPmTfToTf(*this);
}
#endif

PmTf PmTf::inverse() const
{
    PmTf tf;
    tf.stamp_ = stamp_;
    tf.sourceFrameId_ = targetFrameId_;
    tf.targetFrameId_ = sourceFrameId_;
    tf.parameters_ = parameters_.inverse();
    return tf;
}

std::ostream& operator<<(std::ostream& ostream, const PmTf& tf)
{
#ifndef ROS2_BUILD
    ostream << "Stamp: " << tf.stamp_ << "\n";
#else
    ostream << "Stamp: " << tf.stamp_.seconds() << "\n";
#endif
    ostream << "Source frame: " << tf.sourceFrameId_ << "\n";
    ostream << "Target frame: " << tf.targetFrameId_ << "\n";
    ostream << "Parameters:\n" << tf.parameters_ << "\n";
    return ostream;
}

} // namespace pointmatcher_ros