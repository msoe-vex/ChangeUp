#include "swerve_controller/SwerveModule.h"

SwerveModule::SwerveModule(Eigen::Vector2d module_location, double rotation_angle_threshold, 
    double max_velocity, double max_rotation_velocity) : 
    m_module_location(module_location),
    m_rotation_angle_threshold(rotation_angle_threshold), 
    m_max_velocity(max_velocity), 
    m_max_rotation_velocity(max_rotation_velocity) {

}

MotorPowers SwerveModule::InverseKinematics(Eigen::Vector2d targetVelocity, double targetRotationVelocity, Eigen::Rotation2Dd moduleActualAngle) {
    ROS_INFO("Target Velocity - x:%.2f y:%.2f", targetVelocity(0), targetVelocity(1));
    ROS_INFO("Target Rotation Velocity: %.2f", targetRotationVelocity);

    // If you aren't trying to move, make sure to send no velocity to the motors
    if ((targetVelocity(0) == 0) && (targetVelocity(1) == 0) && (targetRotationVelocity == 0)) { //not sure if this works, might need to be reworked
        double scaledMotor1Mag = 0;
        double scaledMotor2Mag = 0;

        MotorPowers motorPowers;
        
        motorPowers.left_motor_power = scaledMotor1Mag;
        motorPowers.right_motor_power = scaledMotor2Mag;

        return motorPowers;
    }

    // Create the maximum vector for each motor
    Eigen::Vector2d maxMotor1Vector(m_max_velocity, 100);
    Eigen::Vector2d maxMotor2Vector(-1 * m_max_velocity, 100);

    // Get the magnitude of the vector (norm returns the magnitude)
    float maxMotor1Mag = maxMotor1Vector.norm() / 2;
    float maxMotor2Mag = maxMotor2Vector.norm() / 2;

    // Take the vector from the origin to the module (module_location) and rotate it to 
    // make it orthogonal to the current (module_location) vector
    Eigen::Vector2d rotatedModuleLocation = Eigen::Rotation2Dd(M_PI / 2) * m_module_location;

    // Multiply the orthogonal vector (rotatedModuleLocation) by the target angular velocity
    // (targetRotationVelocity) to create your target rotation vector
    Eigen::Vector2d targetRotationVector = targetRotationVelocity * rotatedModuleLocation;

    // Add the target velocity and rotation vectors to get a resultant target vector
    Eigen::Vector2d targetVector = targetVelocity + targetRotationVector;

    ROS_INFO("Target Vector - x:%.2f y:%.2f", targetVector(0), targetVector(1));

    // Get the angle of the target vector by taking tangent inverse of y and x components
    // of the vector, and convert to a Rotation2D angle object
    Eigen::Rotation2Dd targetVectorAngle = Eigen::Rotation2Dd(atan2(targetVector(1), targetVector(0)));

    ROS_INFO("Current Angle: %.2f", moduleActualAngle.angle());
    ROS_INFO("Target Angle: %.2f", targetVectorAngle.angle());

    // Subtract the actual module vector from the target to find the change in angle needed
    double moduleRotationDelta = (targetVectorAngle * moduleActualAngle.inverse()).smallestAngle();

    // Determine if we need to only turn the module, or if we can move while turning. Current limit
    // is 60 degrees, if above we will exclusively turn, and if below we turn proportional to the
    // angle needed
    Eigen::Vector2d motorPowerVector;
    if (moduleRotationDelta >= m_rotation_angle_threshold) {
        // Exclusively turn to the target (max speed turn)
        motorPowerVector(1) = m_max_rotation_velocity;
    }
    else {
        // Turn proportional to how far off we are from the target
        motorPowerVector(1) = m_max_rotation_velocity * (moduleRotationDelta / m_rotation_angle_threshold); 
    }

    // Set the power as the magnitude of the vector
    motorPowerVector(0) = targetVector.norm();

    // We are working in the (m/s Forward)-(rpm Speed of Rotation) plane now
    // Project the target vector onto each max motor vector to get components
    // This finds the projection magnitude onto the max motor vector
    double scaledMotor1Mag = motorPowerVector.dot(maxMotor1Vector) / maxMotor1Mag;
    double scaledMotor2Mag = motorPowerVector.dot(maxMotor2Vector) / maxMotor2Mag;

    ROS_INFO("Max Motor 1 Mag: %.2f", maxMotor1Mag);
    ROS_INFO("Max Motor 2 Mag: %.2f", maxMotor2Mag);

    ROS_INFO("Scaled Motor 1 Mag (START): %.2f", scaledMotor1Mag);
    ROS_INFO("Scaled Motor 2 Mag (START): %.2f", scaledMotor2Mag);

    // Find the largest magnitude of the two vectors, and save the scalar
    // The factor of two is to equalize math
    float motorVectorScalar;
    if (scaledMotor1Mag > (maxMotor1Mag * 2) || scaledMotor2Mag > (maxMotor2Mag * 2)) {
        if (scaledMotor1Mag > scaledMotor2Mag) {
            motorVectorScalar = (maxMotor1Mag * 2) / scaledMotor1Mag;
        }
        else {
            motorVectorScalar = (maxMotor2Mag * 2) / scaledMotor2Mag;
        }
    }

    // TODO fix random scale and figure this out
    scaledMotor1Mag /= sqrt(2);
    scaledMotor2Mag /= sqrt(2);

    ROS_INFO("Scaled Motor 1 Mag (1): %.2f", scaledMotor1Mag);
    ROS_INFO("Scaled Motor 2 Mag (1): %.2f", scaledMotor2Mag);

    // Scale both vectors by the magnitude of the largest vector
    scaledMotor1Mag *= motorVectorScalar;
    scaledMotor2Mag *= motorVectorScalar;

    ROS_INFO("Scaled Motor 1 Mag (2): %.2f", scaledMotor1Mag);
    ROS_INFO("Scaled Motor 2 Mag (2): %.2f", scaledMotor2Mag);

    // Scale motors between -127 and 127
    scaledMotor1Mag = (scaledMotor1Mag / 100.0) * 127.0;
    scaledMotor1Mag = (scaledMotor1Mag / 100.0) * 127.0;

    ROS_INFO("Scaled Motor 1 Mag (FINAL): %.2f", scaledMotor1Mag);
    ROS_INFO("Scaled Motor 2 Mag (FINAL): %.2f", scaledMotor2Mag);

    // Set and return the motor powers as a pointer
    MotorPowers motorPowers;

    motorPowers.left_motor_power = (int8_t)scaledMotor1Mag;
    motorPowers.right_motor_power = (int8_t)scaledMotor2Mag;

    return motorPowers;
}
