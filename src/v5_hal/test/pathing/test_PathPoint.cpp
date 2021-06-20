#include <gtest/gtest.h>
#include "pathing/PathPoint.h"

// This is the default fixture for the PathPointTest, intentionally left empty for now
class PathPointTest : public ::testing::Test {
    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

/**
 * @brief Given two points with only constant x velocities, when the first PathPoint is interpolated to the second PathPoint at the second's time, then the resulting PathPoint should be equivalent to the second point.
 * @param PathPointTest The fixture used for testing PathPoints.
 * @param xChangeConstantVel The name of this test.
 * @author Jonathan Phung
 */

TEST(PathPointTest, xChangeConstantVel) {
    PathPoint* firstPoint = new PathPoint(0, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(0)), Eigen::Vector2d(1, 0), 0);
    PathPoint* secondPoint = new PathPoint(1, Pose(Eigen::Vector2d(1, 0), Eigen::Rotation2Dd(0)), Eigen::Vector2d(1, 0), 0);
    PathPoint result = firstPoint->interpolateTo(*secondPoint, 1);

    EXPECT_EQ(secondPoint->getLinearVelocity().x(), result.getLinearVelocity().x());
    EXPECT_EQ(secondPoint->getLinearVelocity().y(), result.getLinearVelocity().y());
    EXPECT_EQ(secondPoint->getPose().position.x(), result.getPose().position.x());
    EXPECT_EQ(secondPoint->getPose().position.y(), result.getPose().position.y());
    EXPECT_EQ(secondPoint->getPose().angle.angle(), result.getPose().angle.angle());
    EXPECT_EQ(secondPoint->getRotationalVelocity(), result.getRotationalVelocity());
    EXPECT_EQ(secondPoint->getTime(), result.getTime());

    delete firstPoint;
    delete secondPoint;
}


/**
 * @brief Given two points with only constant y velocities, when the first PathPoint is interpolated to the second PathPoint at the second's time, then the resulting PathPoint should be equivalent to the second point.
 * @param PathPointTest The fixture used for testing PathPoints.
 * @param yChangeConstantVel The name of this test.
 * @author Jonathan Phung
 */

TEST(PathPointTest, yChangeConstantVel) {
    PathPoint* firstPoint = new PathPoint(0, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(0)), Eigen::Vector2d(0, 1), 0);
    PathPoint* secondPoint = new PathPoint(1, Pose(Eigen::Vector2d(0, 1), Eigen::Rotation2Dd(0)), Eigen::Vector2d(0, 1), 0);
    PathPoint result = firstPoint->interpolateTo(*secondPoint, 1);

    EXPECT_EQ(secondPoint->getLinearVelocity().x(), result.getLinearVelocity().x());
    EXPECT_EQ(secondPoint->getLinearVelocity().y(), result.getLinearVelocity().y());
    EXPECT_EQ(secondPoint->getPose().position.x(), result.getPose().position.x());
    EXPECT_EQ(secondPoint->getPose().position.y(), result.getPose().position.y());
    EXPECT_EQ(secondPoint->getPose().angle.angle(), result.getPose().angle.angle());
    EXPECT_EQ(secondPoint->getRotationalVelocity(), result.getRotationalVelocity());
    EXPECT_EQ(secondPoint->getTime(), result.getTime());

    delete firstPoint;
    delete secondPoint;
}


/**
 * @brief Given two points with only constant x and y velocities, when the first PathPoint is interpolated to the second PathPoint at the second's time, then the resulting PathPoint should be equivalent to the second point.
 * @param PathPointTest The fixture used for testing PathPoints.
 * @param xyChangeConstantVel The name of this test.
 * @author Jonathan Phung
 */

TEST(PathPointTest, xyChangeConstantVel) {
    PathPoint* firstPoint = new PathPoint(0, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(0)), Eigen::Vector2d(1, 1), 0);
    PathPoint* secondPoint = new PathPoint(1, Pose(Eigen::Vector2d(1, 1), Eigen::Rotation2Dd(0)), Eigen::Vector2d(1, 1), 0);
    PathPoint result = firstPoint->interpolateTo(*secondPoint, 1);

    EXPECT_EQ(secondPoint->getLinearVelocity().x(), result.getLinearVelocity().x());
    EXPECT_EQ(secondPoint->getLinearVelocity().y(), result.getLinearVelocity().y());
    EXPECT_EQ(secondPoint->getPose().position.x(), result.getPose().position.x());
    EXPECT_EQ(secondPoint->getPose().position.y(), result.getPose().position.y());
    EXPECT_EQ(secondPoint->getPose().angle.angle(), result.getPose().angle.angle());
    EXPECT_EQ(secondPoint->getRotationalVelocity(), result.getRotationalVelocity());
    EXPECT_EQ(secondPoint->getTime(), result.getTime());

    delete firstPoint;
    delete secondPoint;
}

/**
 * @brief Given two points with no velocity or acceleration, when the first PathPoint is interpolated to the second PathPoint at the second's time, then the resulting PathPoint should be equivalent to the second point.
 * @param PathPointTest The fixture used for testing PathPoints.
 * @param noChange The name of this test.
 * @author Jonathan Phung
 */

TEST(PathPointTest, noChange) {
    PathPoint* firstPoint = new PathPoint(0, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(0)), Eigen::Vector2d(0, 0), 0);
    PathPoint* secondPoint = new PathPoint(1, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(0)), Eigen::Vector2d(0, 0), 0);
    PathPoint result = firstPoint->interpolateTo(*secondPoint, 1);

    EXPECT_EQ(secondPoint->getLinearVelocity().x(), result.getLinearVelocity().x());
    EXPECT_EQ(secondPoint->getLinearVelocity().y(), result.getLinearVelocity().y());
    EXPECT_EQ(secondPoint->getPose().position.x(), result.getPose().position.x());
    EXPECT_EQ(secondPoint->getPose().position.y(), result.getPose().position.y());
    EXPECT_EQ(secondPoint->getPose().angle.angle(), result.getPose().angle.angle());
    EXPECT_EQ(secondPoint->getRotationalVelocity(), result.getRotationalVelocity());
    EXPECT_EQ(secondPoint->getTime(), result.getTime());

    delete firstPoint;
    delete secondPoint;
}