#include "gtest/gtest.h"
#include "TankOdometry.h"

TEST(TankOdometry, StraightLineGyroOdometry) {
    TankOdometry::EncoderConfig encoderConfig;

    encoderConfig.initialTicks = 0;
    encoderConfig.ticksPerWheelRevolution = 1;
    encoderConfig.wheelDiameter = (1 / PI);

    TankOdometry::GetInstance()->Initialize(encoderConfig, encoderConfig);

    //Stationary
    TankOdometry::GetInstance()->Update(0, 0, Rotation2Dd(0));
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.x(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.y(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().angle.angle(), 0, 0.1);

    //Forward
    TankOdometry::GetInstance()->Update(10, 10, Rotation2Dd(0));
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.x(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.y(), 10, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().angle.angle(), 0, 0.1);

    //Backward
    TankOdometry::GetInstance()->Update(0, 0, Rotation2Dd(0));
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.x(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.y(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().angle.angle(), 0, 0.1);

    //180 deg point turn counter-clockwise
    TankOdometry::GetInstance()->Update(-(PI * 10) / 2.0, (PI * 10) / 2.0, Rotation2Dd(PI));
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.x(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.y(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().angle.angle(), PI, 0.1);

    //180 deg point turn clockwise
    TankOdometry::GetInstance()->Update(0, 0, Rotation2Dd(0));
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.x(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.y(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().angle.angle(), 0, 0.1);

    //180 deg point turn clockwise
    TankOdometry::GetInstance()->Update(PI * 10, 0, Rotation2Dd(PI));
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.x(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.y(), -PI * 5, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().angle.angle(), PI, 0.1);


}

TEST(TankOdometry, StraightLineSimpleOdometry) {
    TankOdometry::EncoderConfig encoderConfig;

    encoderConfig.initialTicks = 0;
    encoderConfig.ticksPerWheelRevolution = 1;
    encoderConfig.wheelDiameter = (1 / PI);

    TankOdometry::GetInstance()->Initialize(encoderConfig, encoderConfig);

    //Stationary
    TankOdometry::GetInstance()->Update(0, 0, 10);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.x(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.y(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().angle.angle(), 0, 0.1);

    //Forward
    TankOdometry::GetInstance()->Update(10, 10, 10);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.x(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.y(), 10, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().angle.angle(), 0, 0.1);

    //Backward
    TankOdometry::GetInstance()->Update(0, 0, 10);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.x(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.y(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().angle.angle(), 0, 0.1);

    //180 deg point turn counter-clockwise
    TankOdometry::GetInstance()->Update(-(PI * 10) / 2.0, (PI * 10) / 2.0, 10);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.x(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.y(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().angle.angle(), PI, 0.1);

    //180 deg point turn clockwise
    TankOdometry::GetInstance()->Update(0, 0, 10);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.x(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.y(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().angle.angle(), 0, 0.1);

    //180 deg pivot turn clockwise
    TankOdometry::GetInstance()->Update(PI * 10, 0, 10);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.x(), 0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().position.y(), -(PI * 10) / 2.0, 0.1);
    EXPECT_NEAR(TankOdometry::GetInstance()->GetPose().angle.angle(), -PI, 0.1);
}