#include <gtest/gtest.h>
#include "swerve_controller/SwerveModule.h"


TEST(Test_Vector_Math, DRIVE_FORWARD) {
    // Swerve Module centered in x direction, and 1 meter forward in y direction
    // Threshold for full power rotation of pi/3 (60 deg)
    // Max velocity of 100 m/s, max module rotational speed of 100 RPM
    SwerveModule swerveModule = SwerveModule(Eigen::Vector2d(0, 1), M_PI/3, 100, 100);


    // Drive forwards at full speed, module already facing correct direction
    MotorPowers actualMotorMagnitudes = swerveModule.InverseKinematics(Eigen::Vector2d (0, 100), 0, Eigen::Rotation2Dd (M_PI/2));
    
    MotorPowers expectedMotorMagnitudes;
    expectedMotorMagnitudes.left_motor_power = 127; //expected value
    expectedMotorMagnitudes.right_motor_power = -127; //expected value


    EXPECT_FLOAT_EQ(expectedMotorMagnitudes.left_motor_power, actualMotorMagnitudes.left_motor_power);
    EXPECT_FLOAT_EQ(expectedMotorMagnitudes.right_motor_power, actualMotorMagnitudes.right_motor_power);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}