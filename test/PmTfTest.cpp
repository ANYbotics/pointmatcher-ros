/*
 * PmTfTest.cpp
 *
 * Unit tests for pointmatcher_ros::PmTf ROS transform conversion helpers.
 */

// google test
#include <gtest/gtest.h>

// tf2
#include <tf2/LinearMath/Transform.h>

// pointmatcher_ros
#include "pointmatcher_ros/PmTf.h"

namespace pointmatcher_ros
{
#ifdef ROS2_BUILD
TEST(PmTfTest, IdentityTransform)
{
    constexpr double eps = 1e-6;

    PmTf pmTf;
    pmTf.parameters_.setIdentity();

    const tf2::Transform tf = pmTf.toRosTf();

    EXPECT_NEAR(tf.getOrigin().x(), 0.0, eps);
    EXPECT_NEAR(tf.getOrigin().y(), 0.0, eps);
    EXPECT_NEAR(tf.getOrigin().z(), 0.0, eps);

    const tf2::Vector3 r0 = tf.getBasis().getRow(0);
    const tf2::Vector3 r1 = tf.getBasis().getRow(1);
    const tf2::Vector3 r2 = tf.getBasis().getRow(2);
    EXPECT_NEAR(r0.x(), 1.0, eps);
    EXPECT_NEAR(r0.y(), 0.0, eps);
    EXPECT_NEAR(r0.z(), 0.0, eps);
    EXPECT_NEAR(r1.x(), 0.0, eps);
    EXPECT_NEAR(r1.y(), 1.0, eps);
    EXPECT_NEAR(r1.z(), 0.0, eps);
    EXPECT_NEAR(r2.x(), 0.0, eps);
    EXPECT_NEAR(r2.y(), 0.0, eps);
    EXPECT_NEAR(r2.z(), 1.0, eps);
}

TEST(PmTfTest, TranslationOnly)
{
    constexpr double eps = 1e-6;

    PmTf pmTf;
    pmTf.parameters_.setIdentity();
    pmTf.parameters_(0, 3) = 1.2f;
    pmTf.parameters_(1, 3) = -3.4f;
    pmTf.parameters_(2, 3) = 5.6f;

    const tf2::Transform tf = pmTf.toRosTf();

    EXPECT_NEAR(tf.getOrigin().x(), 1.2, eps);
    EXPECT_NEAR(tf.getOrigin().y(), -3.4, eps);
    EXPECT_NEAR(tf.getOrigin().z(), 5.6, eps);

    const tf2::Vector3 r0 = tf.getBasis().getRow(0);
    const tf2::Vector3 r1 = tf.getBasis().getRow(1);
    const tf2::Vector3 r2 = tf.getBasis().getRow(2);
    EXPECT_NEAR(r0.x(), 1.0, eps);
    EXPECT_NEAR(r0.y(), 0.0, eps);
    EXPECT_NEAR(r0.z(), 0.0, eps);
    EXPECT_NEAR(r1.x(), 0.0, eps);
    EXPECT_NEAR(r1.y(), 1.0, eps);
    EXPECT_NEAR(r1.z(), 0.0, eps);
    EXPECT_NEAR(r2.x(), 0.0, eps);
    EXPECT_NEAR(r2.y(), 0.0, eps);
    EXPECT_NEAR(r2.z(), 1.0, eps);
}

TEST(PmTfTest, RotationAndTranslationBothOverloadsMatch)
{
    constexpr double eps = 1e-6;

    // 90deg yaw about Z + translation.
    PmTf pmTf;
    pmTf.parameters_.setIdentity();
    pmTf.parameters_(0, 0) = 0.0f;
    pmTf.parameters_(0, 1) = -1.0f;
    pmTf.parameters_(1, 0) = 1.0f;
    pmTf.parameters_(1, 1) = 0.0f;
    pmTf.parameters_(2, 2) = 1.0f;
    pmTf.parameters_(0, 3) = 0.5f;
    pmTf.parameters_(1, 3) = 1.0f;
    pmTf.parameters_(2, 3) = -2.0f;

    const tf2::Transform byValue = pmTf.toRosTf();

    tf2::Transform outParam;
    outParam.setOrigin(tf2::Vector3(999.0, 999.0, 999.0));
    outParam.setBasis(tf2::Matrix3x3(9.0,
                                     8.0,
                                     7.0, //
                                     6.0,
                                     5.0,
                                     4.0, //
                                     3.0,
                                     2.0,
                                     1.0));
    pmTf.toRosTf(outParam);

    // Same expected values for both overloads.
    EXPECT_NEAR(byValue.getOrigin().x(), 0.5, eps);
    EXPECT_NEAR(byValue.getOrigin().y(), 1.0, eps);
    EXPECT_NEAR(byValue.getOrigin().z(), -2.0, eps);

    const tf2::Vector3 r0 = byValue.getBasis().getRow(0);
    const tf2::Vector3 r1 = byValue.getBasis().getRow(1);
    const tf2::Vector3 r2 = byValue.getBasis().getRow(2);
    EXPECT_NEAR(r0.x(), 0.0, eps);
    EXPECT_NEAR(r0.y(), -1.0, eps);
    EXPECT_NEAR(r0.z(), 0.0, eps);
    EXPECT_NEAR(r1.x(), 1.0, eps);
    EXPECT_NEAR(r1.y(), 0.0, eps);
    EXPECT_NEAR(r1.z(), 0.0, eps);
    EXPECT_NEAR(r2.x(), 0.0, eps);
    EXPECT_NEAR(r2.y(), 0.0, eps);
    EXPECT_NEAR(r2.z(), 1.0, eps);

    EXPECT_NEAR(outParam.getOrigin().x(), byValue.getOrigin().x(), eps);
    EXPECT_NEAR(outParam.getOrigin().y(), byValue.getOrigin().y(), eps);
    EXPECT_NEAR(outParam.getOrigin().z(), byValue.getOrigin().z(), eps);

    const tf2::Vector3 o0 = outParam.getBasis().getRow(0);
    const tf2::Vector3 o1 = outParam.getBasis().getRow(1);
    const tf2::Vector3 o2 = outParam.getBasis().getRow(2);
    EXPECT_NEAR(o0.x(), r0.x(), eps);
    EXPECT_NEAR(o0.y(), r0.y(), eps);
    EXPECT_NEAR(o0.z(), r0.z(), eps);
    EXPECT_NEAR(o1.x(), r1.x(), eps);
    EXPECT_NEAR(o1.y(), r1.y(), eps);
    EXPECT_NEAR(o1.z(), r1.z(), eps);
    EXPECT_NEAR(o2.x(), r2.x(), eps);
    EXPECT_NEAR(o2.y(), r2.y(), eps);
    EXPECT_NEAR(o2.z(), r2.z(), eps);
}
#endif /* ROS2_BUILD */
} // namespace pointmatcher_ros
