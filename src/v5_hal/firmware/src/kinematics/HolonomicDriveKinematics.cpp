#include "kinematics/HolonomicDriveKinematics.h"

HolonomicDriveKinematics::HolonomicDriveKinematics(EncoderConfig encoder_config, Pose current_pose) :
        IDriveKinematics(encoder_config, current_pose) {

}

float HolonomicDriveKinematics::m_getWheelTicksComponent(float wheel_ticks) {
    return sqrt(pow(wheel_ticks, 2) / 2);
}

void HolonomicDriveKinematics::updateForwardKinematics(IDriveNode::FourMotorDriveEncoderVals encoder_vals) {
    float left_front_dist = encoder_vals.left_front_encoder_val * m_ticks_to_distance_m;
    float left_rear_dist = encoder_vals.left_rear_encoder_val * m_ticks_to_distance_m;
    float right_front_dist = encoder_vals.right_front_encoder_val * m_ticks_to_distance_m;
    float right_rear_dist = encoder_vals.right_rear_encoder_val * m_ticks_to_distance_m;

    if (m_pose_reset) {
        m_left_front_previous_dist = left_front_dist;
        m_left_rear_previous_dist = left_rear_dist;
        m_right_front_previous_dist = right_front_dist;
        m_right_rear_previous_dist = right_rear_dist;
        m_pose_reset = false;
    }

    float left_front_delta = left_front_dist - m_left_front_previous_dist;
    float left_rear_delta = left_rear_dist - m_left_front_previous_dist;
    float right_front_delta = right_front_dist - m_left_front_previous_dist;
    float right_rear_delta = right_rear_dist - m_left_front_previous_dist;
    
    // Positive encoder in the clockwise direction
    float x_dist_moved = (m_getWheelTicksComponent(left_front_delta) -
                          m_getWheelTicksComponent(left_rear_delta) + 
                          m_getWheelTicksComponent(right_front_delta) - 
                          m_getWheelTicksComponent(right_rear_delta)) / 4; 

    // float rotation = (right_delta - left_delta) / track_width;

    // Vector2d robot_translation(0, distance_moved);

    // Odometry::m_robot_pose.angle = Odometry::m_robot_pose.angle * Rotation2Dd(rotation);
    // robot_translation = Odometry::m_robot_pose.angle * robot_translation;
    // Odometry::m_robot_pose.position += robot_translation;

    // Odometry::m_last_encoder_1_dist = left_dist;
    // Odometry::m_last_encoder_2_dist = right_dist;
}

IDriveKinematics::FourMotorPercentages HolonomicDriveKinematics::inverseKinematics(
        float x, float y, float theta, float max) {
    float front_left = y + x + theta;
    float back_left = y - x + theta;
    float front_right = y - x - theta;
    float back_right = y + x - theta;

    float max_val = std::max({fabs(front_left), fabs(back_left), fabs(front_right), fabs(back_right), max});

    FourMotorPercentages motor_percentages {
        front_left / max_val,
        back_left / max_val,
        front_right / max_val,
        back_right / max_val
    };

    return motor_percentages;
}

HolonomicDriveKinematics::~HolonomicDriveKinematics() {

}
