
#include "pointmatcher_ros/PointMatcherFilterIface.h"

#include <exception>
#include <fstream>

namespace PointMatcher_ros {

bool PointMatcherFilterIface::readPipelineFile(const std::string& fileName) {
  std::ifstream inFile(fileName.c_str());
  if (!inFile.good()) {
    ROS_ERROR_STREAM("PointMatcherFilterIface: Couldn't open pipeline description file from \"" << fileName << "\"");
    return false;
  }

  filters_ = PointMatcher<float>::DataPointsFilters(inFile);
  return true;
}

PointMatcher<float>::DataPoints PointMatcherFilterIface::process(const PointMatcher<float>::DataPoints& input) {
  auto localInput = input;

  try {
    filters_.apply(localInput);
  } catch (const std::runtime_error& e) {
    ROS_WARN_STREAM("PointMatcherFilterIface: Caught exception: " << e.what());
    ROS_WARN_STREAM("PointMatcherFilterIface: Point cloud unchanged");
  }

  return localInput;
}

}  // namespace PointMatcher_ros
