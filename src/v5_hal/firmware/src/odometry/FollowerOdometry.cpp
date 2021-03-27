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
    double x_dist = x_encoder_raw_ticks * Odometry::m_encoder_1_ticks_to_dist;
    double y_dist = y_encoder_raw_ticks * Odometry::m_encoder_2_ticks_to_dist;

    // Reset the current position of the robot
    if (m_pose_reset) {
        Odometry::m_last_encoder_1_dist = x_dist;
        Odometry::m_last_encoder_2_dist = y_dist;
        Odometry::m_gyro_initial_angle = gyro_angle;
        Odometry::m_pose_reset = false;
    }

    // Calculate the change in position since the last check
    double x_delta = x_dist - Odometry::m_last_encoder_1_dist;
    double y_delta = y_dist - Odometry::m_last_encoder_2_dist;

    // Convert the x and y deltas into a translation vector
    Vector2d robot_translation(x_delta, y_delta);

    // Update the current angle of the robot position
    Odometry::m_robot_pose.angle = gyro_angle * Odometry::m_gyro_initial_angle.inverse() * Rotation2Dd(m_gyro_offset).inverse();
    //Odometry::m_robot_pose.angle = gyro_angle * Odometry::m_gyro_initial_angle.inverse() * Rotation2Dd(GYRO_OFFSET);

    // Rotate the translation vector by the current angle rotation matrix
    robot_translation = Odometry::m_robot_pose.angle * robot_translation;

    // Add the current translation onto the robot position vector
    Odometry::m_robot_pose.position += robot_translation;

    // Update the previous values of the encoders for the next iteration
    Odometry::m_last_encoder_1_dist = x_dist;
    Odometry::m_last_encoder_2_dist = y_dist;
}