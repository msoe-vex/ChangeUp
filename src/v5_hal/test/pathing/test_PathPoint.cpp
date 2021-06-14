#include <gtest/gtest.h>
#include "pathing/PathPoint.h"

class xChangeConstantVelModule : public ::testing::Test {
    protected:
        PathPoint firstPoint;
        PathPoint secondPoint;

        virtual void SetUp() {
            // Creates two points that only have a change in x with constant velocity
            firstPoint = PathPoint(0, Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd(0)), Eigen::Vector2d(1, 0), 0);
            secondPoint = PathPoint(1, Pose(Eigen::Vector2d(1, 0), Eigen::Rotation2Dd(0)), Eigen::Vector2d(1, 0), 0);
        }

        virtual void TearDown() {

        }
};

TEST_F(xChangeConstantVelModule, xChangeConstantVel) {
    PathPoint result = firstPoint.interpolateTo(secondPoint, 1);
    EXPECT_TRUE(secondPoint.equals(&firstPoint));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}