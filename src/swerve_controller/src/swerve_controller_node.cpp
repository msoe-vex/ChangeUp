#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <ros/console.h>

#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"

Eigen::Vector2d target_velocity;
Eigen::Rotation2Dd actual_angle;
Eigen::Vector2d module_location;

double rotation_velocity;
double x;
double y; 
double rotation_angle_threshold;
double max_velocity;
double max_rotation_velocity;

struct MotorPowers {
    int8_t left_motor_power;
    int8_t right_motor_power;
};

MotorPowers inverseKinematics(Eigen::Vector2d targetVelocity, double targetRotationVelocity, Eigen::Rotation2Dd moduleActualAngle) {
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
    Eigen::Vector2d maxMotor1Vector(max_velocity, 100);
    Eigen::Vector2d maxMotor2Vector(-1 * max_velocity, 100);

    // Get the magnitude of the vector (norm returns the magnitude)
    float maxMotor1Mag = maxMotor1Vector.norm() / 2;
    float maxMotor2Mag = maxMotor2Vector.norm() / 2;

    // Take the vector from the origin to the module (module_location) and rotate it to 
    // make it orthogonal to the current (module_location) vector
    Eigen::Vector2d rotatedModuleLocation = Eigen::Rotation2Dd(M_PI / 2) * module_location;

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
    if (moduleRotationDelta >= rotation_angle_threshold) {
        // Exclusively turn to the target (max speed turn)
        motorPowerVector(1) = max_rotation_velocity;
    }
    else {
        // Turn proportional to how far off we are from the target
        motorPowerVector(1) = max_rotation_velocity * (moduleRotationDelta / rotation_angle_threshold); 
    }

    // Set the power as the magnitude of the vector
    motorPowerVector(0) = targetVector.norm();

    // We are working in the (m/s Forward)-(rpm Speed of Rotation) plane now
    // Project the target vector onto each max motor vector to get components
    // This finds the projection magnitude onto the max motor vector
    double scaledMotor1Mag = motorPowerVector.dot(maxMotor1Vector) / maxMotor1Mag;
    double scaledMotor2Mag = motorPowerVector.dot(maxMotor2Vector) / maxMotor2Mag;

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

    // Scale both vectors by the magnitude of the largest vector
    scaledMotor1Mag *= motorVectorScalar;
    scaledMotor2Mag *= motorVectorScalar;

    // Scale motors between -127 and 127
    scaledMotor1Mag = (scaledMotor1Mag / 100.0) * 127.0;
    scaledMotor1Mag = (scaledMotor1Mag / 100.0) * 127.0;

    // Set and return the motor powers as a pointer
    MotorPowers motorPowers;

    motorPowers.left_motor_power = (int8_t)scaledMotor1Mag;
    motorPowers.right_motor_power = (int8_t)scaledMotor2Mag;

    return motorPowers;
}

void assignTargetVelocity(const geometry_msgs::Vector3& msg) {
    target_velocity(0) = msg.x;
    target_velocity(1) = msg.y;
}

void assignRotationVelocity(const std_msgs::Float32& msg) {
    rotation_velocity = msg.data;
}

void assignActualAngle(const std_msgs::Float32& msg) {
    actual_angle = Eigen::Rotation2Dd(msg.data);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "swerve_controller");

    ros::NodeHandle handle;

    handle.param("module_x", x, 5.0);
    handle.param("module_y", y, 5.0);
    module_location(0) = x;
    module_location(1) = y;

    handle.param("rotation_angle_threshold", rotation_angle_threshold, (M_PI / 3));
    handle.param("max_velocity", max_velocity, 1.31);
    handle.param("max_rotation_velocity", max_rotation_velocity, 100.0);

    ros::Subscriber swerve_controller_tf_sub = handle.subscribe("swerveCommandTf", 10, assignTargetVelocity);
    ros::Subscriber swerve_controller_rotate_sub = handle.subscribe("swerveCommandRotate", 10, assignRotationVelocity);
    ros::Subscriber swerve_controller_angle_sub = handle.subscribe("swerveCommandActAngle", 10, assignActualAngle);

    ros::Publisher swerve_controller_left_motor_pub = handle.advertise<std_msgs::Int8>("leftMotor", 10);
    ros::Publisher swerve_controller_right_motor_pub = handle.advertise<std_msgs::Int8>("rightMotor", 10);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        MotorPowers motor_powers = inverseKinematics(target_velocity, rotation_velocity, actual_angle);

        std_msgs::Int8 left_motor_power;
        std_msgs::Int8 right_motor_power;

        left_motor_power.data = motor_powers.left_motor_power;
        right_motor_power.data = motor_powers.right_motor_power;

        swerve_controller_left_motor_pub.publish(left_motor_power);
        swerve_controller_right_motor_pub.publish(right_motor_power);

        ros::spinOnce();
        loop_rate.sleep();
    }
}