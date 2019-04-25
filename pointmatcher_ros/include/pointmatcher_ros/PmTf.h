#pragma once

// std
#include <string>

// ros
#include <ros/ros.h>

// geometry msgs
#include <geometry_msgs/TransformStamped.h>

// tf
#pragma GCC diagnostic push // Get rid of compiler warnings we can't fix.
#pragma GCC diagnostic ignored "-Wpedantic"
#include <tf/tf.h>
#pragma GCC diagnostic pop // Back to normal warning handling.

// pointmatcher_ros
#include "pointmatcher_ros/usings.h"

namespace PointMatcher_ros {

class PmTf {
 public:
  ros::Time stamp_;
  std::string sourceFrameId_;
  std::string targetFrameId_;
  PmTfParameters parameters_;

 protected:
  std::shared_ptr<PmTransformator> transformator_;

 public:
  PmTf();

  static PmTf FromRosTfMsg(const geometry_msgs::TransformStamped& tfMsg);
  void fromRosTfMsg(const geometry_msgs::TransformStamped& tfMsg);
  geometry_msgs::TransformStamped toRosTfMsg() const;
  void toRosTfMsg(geometry_msgs::TransformStamped& tfMsg) const;

  static PmTf FromRosTf(const tf::StampedTransform& tf);
  void fromRosTf(const tf::StampedTransform& tf);
  tf::StampedTransform toRosTf() const;
  void toRosTf(tf::StampedTransform& tf) const;

  PmTf inverse() const;

  float getRotationScaling() const;
  void fixRotationScaling();
};

std::ostream& operator<<(std::ostream& ostream, const PmTf& tf);

}  // namespace PointMatcher_ros
