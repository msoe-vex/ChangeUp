#include <gtest/gtest.h>
#include "pathing/PathPoint.h"

class xChangeConstantVelModule : public ::testing::Test {
    protected:
        PathPoint* firstPoint;
        PathPoint* secondPoint;

        virtual void SetUp() {
            // Creates two points that only have a positive change in x with constant velocity
            firstPoint = new PathPoint(0, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(0)), Eigen::Vector2d(1, 0), 0);
            secondPoint = new PathPoint(1, Pose(Eigen::Vector2d(1, 0), Eigen::Rotation2Dd(0)), Eigen::Vector2d(1, 0), 0);
        }

        virtual void TearDown() {
            delete firstPoint;
            delete secondPoint;
        }
};

class yChangeConstantVelModule : public ::testing::Test {
    protected:
        PathPoint* firstPoint;
        PathPoint* secondPoint;

        virtual void SetUp() {
            // Creates two points that only have a positive change in y with constant velocity
            firstPoint = new PathPoint(0, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(0)), Eigen::Vector2d(0, 1), 0);
            secondPoint = new PathPoint(1, Pose(Eigen::Vector2d(0, 1), Eigen::Rotation2Dd(0)), Eigen::Vector2d(0, 1), 0);
        }

        virtual void TearDown() {
            delete firstPoint;
            delete secondPoint;
        }
};

class xyChangeConstantVelModule : public ::testing::Test {
    protected:
        PathPoint* firstPoint;
        PathPoint* secondPoint;

        virtual void SetUp() {
            // Creates two points that have a positive change in x and y with constant velocity
            firstPoint = new PathPoint(0, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(0)), Eigen::Vector2d(1, 1), 0);
            secondPoint = new PathPoint(1, Pose(Eigen::Vector2d(1, 1), Eigen::Rotation2Dd(0)), Eigen::Vector2d(1, 1), 0);
        }

        virtual void TearDown() {
            delete firstPoint;
            delete secondPoint;
        }
};

class noChangeModule : public ::testing::Test {
    protected:
        PathPoint* firstPoint;
        PathPoint* secondPoint;

        virtual void SetUp() {
            // Creates two points that do not change
            firstPoint = new PathPoint(0, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(0)), Eigen::Vector2d(0, 0), 0);
            secondPoint = new PathPoint(1, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(0)), Eigen::Vector2d(0, 0), 0);
        }

        virtual void TearDown() {
            delete firstPoint;
            delete secondPoint;
        }
};

TEST_F(xChangeConstantVelModule, xChangeConstantVel) {
    PathPoint result = firstPoint->interpolateTo(*secondPoint, 1);

    EXPECT_EQ(secondPoint->getLinearVelocity().x(), result.getLinearVelocity().x());
    EXPECT_EQ(secondPoint->getLinearVelocity().y(), result.getLinearVelocity().y());
    EXPECT_EQ(secondPoint->getPose().position.x(), result.getPose().position.x());
    EXPECT_EQ(secondPoint->getPose().position.y(), result.getPose().position.y());
    EXPECT_EQ(secondPoint->getPose().angle.angle(), result.getPose().angle.angle());
    EXPECT_EQ(secondPoint->getRotationalVelocity(), result.getRotationalVelocity());
    EXPECT_EQ(secondPoint->getTime(), result.getTime());
}

TEST_F(yChangeConstantVelModule, yChangeConstantVel) {
    PathPoint result = firstPoint->interpolateTo(*secondPoint, 1);

    EXPECT_EQ(secondPoint->getLinearVelocity().x(), result.getLinearVelocity().x());
    EXPECT_EQ(secondPoint->getLinearVelocity().y(), result.getLinearVelocity().y());
    EXPECT_EQ(secondPoint->getPose().position.x(), result.getPose().position.x());
    EXPECT_EQ(secondPoint->getPose().position.y(), result.getPose().position.y());
    EXPECT_EQ(secondPoint->getPose().angle.angle(), result.getPose().angle.angle());
    EXPECT_EQ(secondPoint->getRotationalVelocity(), result.getRotationalVelocity());
    EXPECT_EQ(secondPoint->getTime(), result.getTime());
}

TEST_F(xyChangeConstantVelModule, xyChangeConstantVel) {
    PathPoint result = firstPoint -> interpolateTo(*secondPoint, 1);

    EXPECT_EQ(secondPoint->getLinearVelocity().x(), result.getLinearVelocity().x());
    EXPECT_EQ(secondPoint->getLinearVelocity().y(), result.getLinearVelocity().y());
    EXPECT_EQ(secondPoint->getPose().position.x(), result.getPose().position.x());
    EXPECT_EQ(secondPoint->getPose().position.y(), result.getPose().position.y());
    EXPECT_EQ(secondPoint->getPose().angle.angle(), result.getPose().angle.angle());
    EXPECT_EQ(secondPoint->getRotationalVelocity(), result.getRotationalVelocity());
    EXPECT_EQ(secondPoint->getTime(), result.getTime());
}

TEST_F(noChangeModule, noChange) {
    PathPoint result = firstPoint -> interpolateTo(*secondPoint, 1);

    EXPECT_EQ(secondPoint->getLinearVelocity().x(), result.getLinearVelocity().x());
    EXPECT_EQ(secondPoint->getLinearVelocity().y(), result.getLinearVelocity().y());
    EXPECT_EQ(secondPoint->getPose().position.x(), result.getPose().position.x());
    EXPECT_EQ(secondPoint->getPose().position.y(), result.getPose().position.y());
    EXPECT_EQ(secondPoint->getPose().angle.angle(), result.getPose().angle.angle());
    EXPECT_EQ(secondPoint->getRotationalVelocity(), result.getRotationalVelocity());
    EXPECT_EQ(secondPoint->getTime(), result.getTime());
}