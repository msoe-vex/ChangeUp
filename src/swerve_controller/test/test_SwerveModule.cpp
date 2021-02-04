#include <gtest/gtest.h>
#include "swerve_controller/SwerveModule.h"

class SimpleForwardModule : public ::testing::Test {
protected:
    SwerveModule * swerveModule;

    virtual void SetUp() {
        // Swerve Module centered in x direction, and 1 meter forward in y direction
        // Threshold for full power rotation of pi/3 (60 deg)
        // Max velocity of 100 m/s, max module rotational speed of 100 RPM
        swerveModule = new SwerveModule(Eigen::Vector2d(0, 1), M_PI/3, 100, 100);
    }

    virtual void TearDown() {
        delete swerveModule;
    }
};

TEST_F(SimpleForwardModule, DriveForwardFullSpeed) {
    // Drive forwards at full speed, module already facing correct direction
    MotorPowers motorMags = swerveModule->InverseKinematics(Eigen::Vector2d(0, 100), 0, Eigen::Rotation2Dd(M_PI/2));

    EXPECT_EQ(motorMags.left_motor_power, 127);
    EXPECT_EQ(motorMags.right_motor_power, -127);
}

TEST_F(SimpleForwardModule, DriveForwardHalfSpeed) {
    // Drive forwards at full speed, module already facing correct direction
    MotorPowers motorMags = swerveModule->InverseKinematics(Eigen::Vector2d(0, 50), 0, Eigen::Rotation2Dd(M_PI/2));

    EXPECT_EQ(motorMags.left_motor_power, 127/2);
    EXPECT_EQ(motorMags.right_motor_power, -127/2);
}

TEST_F(SimpleForwardModule, RotateInPlaceFullPower) {
    // Drive forwards at full speed, module already facing correct direction
    MotorPowers motorMags = swerveModule->InverseKinematics(Eigen::Vector2d(0, 0), 1, Eigen::Rotation2Dd(0));

    EXPECT_EQ(motorMags.left_motor_power, 127);
    EXPECT_EQ(motorMags.right_motor_power, 127);
}

TEST_F(SimpleForwardModule, RotateInPlaceHalfPower) {
    // Drive forwards at full speed, module already facing correct direction
    MotorPowers motorMags = swerveModule->InverseKinematics(Eigen::Vector2d(0, 0), 50, Eigen::Rotation2Dd(M_PI - M_PI/6));

    EXPECT_NEAR(motorMags.left_motor_power, 127, 1);
    EXPECT_EQ(motorMags.right_motor_power, 0);
}

TEST_F(SimpleForwardModule, OverSpeedNegativeDirection) {
    // Drive forwards at full speed, module already facing correct direction
    MotorPowers motorMags = swerveModule->InverseKinematics(Eigen::Vector2d(0, -1000), 0, Eigen::Rotation2Dd(-M_PI/2));

    EXPECT_EQ(motorMags.left_motor_power, 127);
    EXPECT_EQ(motorMags.right_motor_power, -127);
}

TEST_F(SimpleForwardModule, SetModuleToAngleNoForwardSpeed) {
    // Drive forwards at full speed, module already facing correct direction
    MotorPowers motorMags = swerveModule->InverseKinematics(Eigen::Vector2d(0, 0.00000000001), 0, Eigen::Rotation2Dd(M_PI/3));

    EXPECT_EQ(motorMags.left_motor_power, 63);
    EXPECT_EQ(motorMags.right_motor_power, 63);
}


int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}