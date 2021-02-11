#include "gtest/gtest.h"
#include "PathManager.h"
#include "Math/Pose.h"
#include "TankOdometry.h"
#include "AdaptivePursuit.h"
#include <stdio.h>
#include "testPaths/simplePath.h"

TEST(AdaptivePurePursuit, FollowsStraightPath) {
    PathManager::GetInstance()->LoadPathsText(simplePath);

    auto paths = PathManager::GetInstance()->GetPaths();

    AdaptivePursuit controller(10, 0.1, 0.1, paths["FirstPath"], false,
            0.01, true, 3.25);

    TankOdometry::EncoderConfig encoderConfig;

    encoderConfig.initialTicks = 0;
    encoderConfig.ticksPerWheelRevolution = 360;
    encoderConfig.wheelDiameter = 3.25;

    TankOdometry::GetInstance()->Initialize(encoderConfig, encoderConfig,
                                            Pose(Vector2d(paths["FirstPath"].getFirstWaypoint().position.getY(),-paths["FirstPath"].getFirstWaypoint().position.getX()), Rotation2Dd(0)));

    double leftPosition = 0;
    double rightPosition = 0;

    const double dt = 0.1;

    FILE *myfile;

    myfile = fopen("log.csv", "w");

    fprintf(myfile, "time,leftVelocity,rightVelocity,leftPosition,rightPosition,x,y,\n");

    for (double t = dt; t < 25; t += dt) {
        auto command = controller.Update(TankOdometry::GetInstance()->GetPose(), t);

        leftPosition += (command.left * dt) * 100 / 60 * 360;
        rightPosition += (command.right * dt) * 100 / 60 * 360;

        TankOdometry::GetInstance()->Update(leftPosition, rightPosition, 10);

        fprintf(myfile, "%.1f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,\n", t, command.left, command.right,
                leftPosition, rightPosition, TankOdometry::GetInstance()->GetPose().position.x(),
                TankOdometry::GetInstance()->GetPose().position.y());
    }

    fclose(myfile);

    Pose pose = TankOdometry::GetInstance()->GetPose();

    EXPECT_NEAR(pose.position.y(), 71.0, 0.1);
    EXPECT_NEAR(pose.position.x(), 0.0, 0.02);
}

TEST(AdaptivePurePursuit, FollowsSplinePath) {
    PathManager::GetInstance()->LoadPathsText(simplePath);

    auto paths = PathManager::GetInstance()->GetPaths();

    AdaptivePursuit controller(6, 10,0.1, paths["TestPath"], false,
                               0.01, true, 4);

    TankOdometry::EncoderConfig encoderConfig;

    encoderConfig.initialTicks = 0;
    encoderConfig.ticksPerWheelRevolution = 1;
    encoderConfig.wheelDiameter = (1 / PI);

    TankOdometry::GetInstance()->Initialize(encoderConfig, encoderConfig,
                                            Pose(Vector2d(paths["TestPath"].getFirstWaypoint().position.getY(),-paths["TestPath"].getFirstWaypoint().position.getX()), Rotation2Dd(0)));

    double leftPosition = 0;
    double rightPosition = 0;

    const double dt = 0.1;

    FILE *myfile;

    myfile = fopen("log2.csv", "w");

    fprintf(myfile, "time,leftVelocity,rightVelocity,leftPosition,rightPosition,x,y,\n");

    for (double t = dt; t < 30; t += dt) {
        auto command = controller.Update(TankOdometry::GetInstance()->GetPose(), t);

        leftPosition += (command.left * dt);
        rightPosition += (command.right * dt);

        TankOdometry::GetInstance()->Update(leftPosition, rightPosition, 10);

        fprintf(myfile, "%.1f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,\n", t, command.left, command.right,
                leftPosition, rightPosition, TankOdometry::GetInstance()->GetPose().position.x(),
                TankOdometry::GetInstance()->GetPose().position.y());
    }

    fclose(myfile);

    Pose pose = TankOdometry::GetInstance()->GetPose();

    EXPECT_NEAR(pose.position.x(), 30, 0.3);
    EXPECT_NEAR(pose.position.y(), 70, 0.3);
}