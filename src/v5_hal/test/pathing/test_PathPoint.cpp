#include <gtest/gtest.h>
#include "pathing/PathPoint.h"

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

/**
 * @brief Given four points, all 90 degrees clockwise apart, the path should correctly interpolate between them
 * @param PathPointTest The fixture used for testing PathPoints.
 * @param rotateInPlace The name of this test.
 * @author Nathan DuPont
 */
TEST(PathPointTest, rotateInPlace) {
    PathPoint* pointAt0Degrees = new PathPoint(0, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(0)), Eigen::Vector2d(0, 0), 0.1);
    PathPoint* pointAt90Degrees = new PathPoint(1, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(M_PI_2)), Eigen::Vector2d(0, 0), 0.1);
    PathPoint* pointAt180Degrees = new PathPoint(2, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(M_PI)), Eigen::Vector2d(0, 0), 0.1);
    PathPoint* pointAt270Degrees = new PathPoint(3, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(3 * M_PI_2)), Eigen::Vector2d(0, 0), 0.1);
    PathPoint* pointAt360Degrees = new PathPoint(3, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(M_2_PI)), Eigen::Vector2d(0, 0), 0.1);
    
    PathPoint testPointAt45Degrees = pointAt0Degrees->interpolateTo(*pointAt90Degrees, 0.5);

    EXPECT_EQ(testPointAt45Degrees.getLinearVelocity().x(), 0.);
    EXPECT_EQ(testPointAt45Degrees.getLinearVelocity().y(), 0.);
    EXPECT_EQ(testPointAt45Degrees.getPose().position.x(), 0.);
    EXPECT_EQ(testPointAt45Degrees.getPose().position.y(), 0.);
    EXPECT_NEAR(testPointAt45Degrees.getPose().angle.angle(), M_PI_4, 0.001);
    EXPECT_EQ(testPointAt45Degrees.getRotationalVelocity(), 0.1);
    EXPECT_EQ(testPointAt45Degrees.getTime(), 0.5);

    delete pointAt0Degrees;
    delete pointAt90Degrees;
    delete pointAt180Degrees;
    delete pointAt270Degrees;
    delete pointAt360Degrees;
}