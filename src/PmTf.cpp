
#include "pointmatcher_ros/PmTf.h"

// pointmatcher_ros
#include "pointmatcher_ros/helper_functions.h"

namespace pointmatcher_ros
{

PmTf::PmTf() : transformator_(std::shared_ptr<PmTransformator>(Pm::get().REG(Transformation).create("RigidTransformation")))
{
    parameters_ = Pm::Matrix::Identity(4, 4);
}

PmTf PmTf::FromRosTfMsg(const geometry_msgs::TransformStamped& tfMsg)
{
    PmTf pmTf;
    pmTf.fromRosTfMsg(tfMsg);
    return pmTf;
}

void PmTf::fromRosTfMsg(const geometry_msgs::TransformStamped& tfMsg)
{
    *this = convertRosTfMsgToPmTf(tfMsg);
}

geometry_msgs::TransformStamped PmTf::toRosTfMsg() const
{
    geometry_msgs::TransformStamped tfMsg;
    toRosTfMsg(tfMsg);
    return tfMsg;
}

void PmTf::toRosTfMsg(geometry_msgs::TransformStamped& tfMsg) const
{
    tfMsg = convertPmTfToTfMsg(*this);
}

PmTf PmTf::FromRosTf(const tf::StampedTransform& tf)
{
    PmTf pmTf;
    pmTf.fromRosTf(tf);
    return pmTf;
}

void PmTf::fromRosTf(const tf::StampedTransform& tf)
{
    *this = convertRosTfToPmTf(tf);
}

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

PmTf PmTf::inverse() const
{
    PmTf tf;
    tf.stamp_ = stamp_;
    tf.sourceFrameId_ = targetFrameId_;
    tf.targetFrameId_ = sourceFrameId_;
    tf.parameters_ = parameters_.inverse();
    return tf;
}

float PmTf::getRotationScaling() const
{
    return parameters_.topLeftCorner<3, 3>().determinant();
}

void PmTf::fixRotationScaling()
{
    transformator_->correctParameters(parameters_);
}

std::ostream& operator<<(std::ostream& ostream, const PmTf& tf)
{
    ostream << "Stamp: " << tf.stamp_ << "\n";
    ostream << "Source frame: " << tf.sourceFrameId_ << "\n";
    ostream << "Target frame: " << tf.targetFrameId_ << "\n";
    ostream << "Parameters:\n" << tf.parameters_ << "\n";
    return ostream;
}

} // namespace pointmatcher_ros