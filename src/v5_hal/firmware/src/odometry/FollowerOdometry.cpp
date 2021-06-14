#include "odometry/FollowerOdometry.h"

FollowerOdometry::FollowerOdometry(EncoderConfig xEncoderConfig, EncoderConfig yEncoderConfig, 
    Pose currentPose): Odometry(xEncoderConfig, yEncoderConfig, currentPose) {

}

/**
 * This function is used to update the internal calculated position of the robot
 *
 * To maximize accuracy in robot position estimation, this function should be called as frequently as possible, ideally
 * at a rate of 50 Hz or greater.
 *
 * @param x_encoder_raw_ticks The distance in ticks measured by the X encoder since when the tracker was last run
 * @param y_encoder_raw_ticks The distance in ticks measured by the Y encoder since when the tracker was last run
 * @param gyro_angle The current yaw of the robot as measured by the gyro
 */
void FollowerOdometry::Update(double x_encoder_raw_ticks, double y_encoder_raw_ticks, Rotation2Dd gyro_angle) {
    // Convert the current position in ticks to a position in distance units
    double x_encoder_dist = x_encoder_raw_ticks * Odometry::m_encoder_1_ticks_to_dist;
    double y_encoder_dist = y_encoder_raw_ticks * Odometry::m_encoder_2_ticks_to_dist;

    Vector2d x_encoder_location(ODOM_PERPENDICULAR_X, ODOM_PERPENDICULAR_Y);
    Vector2d y_encoder_location(ODOM_PARALLEL_X, ODOM_PARALLEL_Y);

    // Reset the current position of the robot
    if (m_pose_reset) {
        Odometry::m_last_encoder_1_dist = x_encoder_dist;
        Odometry::m_last_encoder_2_dist = y_encoder_dist;
        Odometry::m_gyro_initial_angle = gyro_angle;
        Odometry::m_pose_reset = false;
    }

    // Calculate the change in position of each encoder since the last check
    double x_encoder_delta = x_encoder_dist - Odometry::m_last_encoder_1_dist;
    double y_encoder_delta = y_encoder_dist - Odometry::m_last_encoder_2_dist;
    Rotation2Dd angle_delta = gyro_angle * m_robot_pose.angle.inverse(); // Find change in angle

    double x_arc_length = x_encoder_location.norm() * angle_delta.angle();
    double y_arc_length = y_encoder_location.norm() * angle_delta.angle();

    // Need to create a dist to ticks
    // double x_arc_length_ticks = x_arc_length

    double x_position_coef = fabs(sin(atan(x_encoder_location.y() / x_encoder_location.x()))); // 48.14252608888209 for 90 deg
    double y_position_coef = fabs(cos(atan(y_encoder_location.y() / y_encoder_location.x()))); // 28.99733373232959 for 90 deg

    // -6.8475
    // -1.42578274 rad
    // -0.98950394259

    // 2.2422857
    // 1.15129588 rad
    // 0.40730426569

    // Translate to encoder value
    double x_encoder_turning_component = x_arc_length * x_position_coef;
    double y_encoder_turning_component = y_arc_length * y_position_coef;

    double x_delta = x_encoder_delta * x_encoder_turning_component;
    double y_delta = y_encoder_delta * y_encoder_turning_component;

    double calculated_x_coef = x_encoder_dist / (x_encoder_location.norm() * gyro_angle.angle()); 
    double calculated_y_coef = y_encoder_dist / (y_encoder_location.norm() * gyro_angle.angle()); 

    Logger::logInfo("x_encoder_dist: " + std::to_string(x_encoder_dist) + 
                    " | x_encoder_turning_component: " + std::to_string(x_encoder_turning_component) +
                    " | x_coefficient: " + std::to_string(calculated_x_coef) +
                    " | y_encoder_dist: " + std::to_string(y_encoder_dist) + 
                    " | y_encoder_turning_component: " + std::to_string(y_encoder_turning_component) +
                    " | y_coefficient: " + std::to_string(calculated_y_coef) + 
                    " | angle:" + std::to_string(gyro_angle.angle()));

    // total_x = total_x + x_encoder_delta;
    // total_y = total_y + y_encoder_delta;

    // turn_x = turn_x + (x_encoder_location.norm() * angle_delta.angle() * fabs(sin(atan(x_encoder_location.y() / x_encoder_location.x()))));
    // turn_y = turn_y + (y_encoder_location.norm() * angle_delta.angle() * fabs(cos(atan(y_encoder_location.y() / y_encoder_location.x()))));

    // // Logger::logInfo("Encoder: " + std::to_string(x_encoder_delta) + 
    // //                 " | Modified: " + std::to_string(x_delta) + 
    // //                 " | Change in angle: " + std::to_string(angle_delta.angle()));

    // Logger::logInfo("Total X: " + std::to_string(total_x) + 
    //                 " | Total Y: " + std::to_string(total_y) + 
    //                 " | Turn X: " + std::to_string(turn_x) + 
    //                 " | Turn Y: " + std::to_string(turn_y));

    // Convert the x and y deltas into a translation vector
    Vector2d robot_translation(x_encoder_delta, y_encoder_delta);

    // Update the current angle of the robot position
    // Find the difference of the current angle to the initial angle
    // Rotate this difference by the M_PI_2 offset to put it back in the correct frame of reference
    Odometry::m_robot_pose.angle = gyro_angle * Odometry::m_gyro_initial_angle.inverse() * Odometry::m_gyro_offset;

    // Rotate the translation vector by the current angle rotation matrix
    // We need to rotate this back, as our encoders are oriented with 0 being forward
    robot_translation = (Odometry::m_robot_pose.angle * Odometry::m_gyro_offset.inverse()) * robot_translation;

    // Add the current translation onto the robot position vector
    Odometry::m_robot_pose.position += robot_translation;

    // Update the previous values of the encoders for the next iteration
    Odometry::m_last_encoder_1_dist = x_encoder_dist;
    Odometry::m_last_encoder_2_dist = y_encoder_dist;
}