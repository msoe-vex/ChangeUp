#include "swerve_controller/SwerveModule.h"
#include <iostream>

SwerveModule::SwerveModule(Eigen::Vector2d moduleLocation, double rotationAngleThreshold, double maxVelocity, double maxRotationVelocity) {
    m_moduleLocation = moduleLocation;
    m_rotationAngleThreshold = rotationAngleThreshold;
    m_maxVelocity = maxVelocity;
    m_maxRotationVelocity = maxRotationVelocity;
}

MotorPowers* SwerveModule::InverseKinematics(Eigen::Vector2d targetVelocity, double targetRotationVelocity, Eigen::Rotation2Dd moduleActualAngle) {
    // If you aren't trying to move, make sure to send no velocity to the motors
    if ((targetVelocity(0) == 0) && (targetVelocity(1) == 0) && (targetRotationVelocity == 0)) { //not sure if this works, might need to be reworked
        double scaledMotor1Mag = 0;
        double scaledMotor2Mag = 0;

        MotorPowers* motorPowers = new MotorPowers;
        motorPowers->left_motor_power = scaledMotor1Mag;
        motorPowers->right_motor_power = scaledMotor2Mag;

        return motorPowers;
    }

    // Create the maximum vector for each motor
    Eigen::Vector2d maxMotor1Vector(m_maxVelocity, 100);
    Eigen::Vector2d maxMotor2Vector(-1 * m_maxVelocity, 100);

    // Get the magnitude of the vector (norm returns the magnitude)
    float maxMotor1Mag = maxMotor1Vector.norm() / 2;
    float maxMotor2Mag = maxMotor2Vector.norm() / 2;

    // Inverting the current module location and applying a rotation vector
    Eigen::Vector2d rotatedModuleLocation = Eigen::Rotation2Dd(M_PI / 2) * m_moduleLocation;

    // Get the rotation vector for the module
    Eigen::Vector2d targetRotationVector = targetRotationVelocity * rotatedModuleLocation;

    // Create a resultant vector from the velocity and rotation
    Eigen::Vector2d targetVector = targetVelocity + targetRotationVector;

    // Get the angle of the target vector
    Eigen::Rotation2Dd targetVectorAngle = Eigen::Rotation2Dd(atan2(targetVector(1), targetVector(0)));

    // Determine the change in rotation for the module
    double moduleRotationDelta = (targetVectorAngle * moduleActualAngle.inverse()).smallestAngle();

    Eigen::Vector2d motorPowerVector;
    if (moduleRotationDelta >= m_rotationAngleThreshold) {
        motorPowerVector(1) = m_maxRotationVelocity; // Make sure the rotation vector isn't over the max
    }
    else {
        motorPowerVector(1) = m_maxRotationVelocity * (moduleRotationDelta / m_rotationAngleThreshold); // Apply rotation vector
    }

    // Set the power as the magnitude of the vector
    motorPowerVector(0) = targetVector.norm();

    // Get the magnitude of the scaled motor vectors
    double scaledMotor1Mag = motorPowerVector.dot(maxMotor1Vector) / maxMotor1Mag;
    double scaledMotor2Mag = motorPowerVector.dot(maxMotor2Vector) / maxMotor2Mag;

    // Find the largest magnitude of the two vectors, and save the scalar
    float motorVectorScalar = 1;
    if (scaledMotor1Mag > (maxMotor1Mag * 2)) {
        if (scaledMotor1Mag > scaledMotor2Mag) {
            motorVectorScalar = (maxMotor1Mag * 2) / scaledMotor1Mag;
        }
        else {
            motorVectorScalar = (maxMotor2Mag * 2) / scaledMotor2Mag;
        }
    }

    // Normalize the vectors
    scaledMotor1Mag /= sqrt(2);
    scaledMotor2Mag /= sqrt(2);

    // Scale both vectors by the magnitude of the largest vector
    scaledMotor1Mag *= motorVectorScalar;
    scaledMotor2Mag *= motorVectorScalar;

    // Scale motors between -127 and 127
    scaledMotor1Mag = (scaledMotor1Mag / 100) * 127;
    scaledMotor1Mag = (scaledMotor1Mag / 100) * 127;

    // Set and return the motor powers as a pointer
    MotorPowers* motorPowers = new MotorPowers;

    motorPowers->left_motor_power = (int8_t)scaledMotor1Mag;
    motorPowers->right_motor_power = (int8_t)scaledMotor2Mag;

    return motorPowers;
}