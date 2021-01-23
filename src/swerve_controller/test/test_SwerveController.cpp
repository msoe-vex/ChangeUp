#include <gtest/gtest.h>
#include "swerve_controller/SwerveController.h"



TEST(Test_Vector_Math, Tangential_Velocity) {
    
    SwerveModule swerveModule = SwerveModule (Eigen::Vector2d (0, 2), M_PI/3, 100, 100);

    motorMagnitudes expectedMotorMagnitudes = motorMagnitudes ();

    expectedMotorMagnitudes.motor1Magnitude = sqrt(2); //expected value
    expectedMotorMagnitudes.motor2Magnitude = -sqrt(2); //expected value

    motorMagnitudes* actualMotorMagnitudes = swerveModule.InverseKinematics(Eigen::Vector2d (0, 0), 0.0, Eigen::Rotation2Dd (M_PI/2));

    EXPECT_FLOAT_EQ(expectedMotorMagnitudes.motor1Magnitude, actualMotorMagnitudes->motor1Magnitude);
    EXPECT_FLOAT_EQ(expectedMotorMagnitudes.motor2Magnitude, actualMotorMagnitudes->motor2Magnitude);

    delete actualMotorMagnitudes;
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}