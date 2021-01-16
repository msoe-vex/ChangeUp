#include <gtest/gtest.h>
#include "swerve_controller/SwerveController.h"



TEST(Test_Vector_Math, Tangential_Velocity) {

    SwerveModule swerveModule = SwerveModule (Eigen::Vector2d (0, 2), 60.0, 1, 1.31, );

    motorVectors expectedMotorVectors = motorVectors;

    expectedMotorVectors.motor1Vector = //expected value

    motorVector* actualMotorVectors = SwerveModule::InverseKinematics();

    EXPECT_EQ(expectedMotorVectors.motor1Vector, *actualMotorVectors.motor1Vector);

    delete ptr;
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}