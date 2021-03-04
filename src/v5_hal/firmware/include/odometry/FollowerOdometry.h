#pragma once

#include "odometry/Odometry.h"

class FollowerOdometry : public Odometry {
private:
    Eigen::Rotation2Dd m_gyro_offset;
    
public:
    FollowerOdometry(EncoderConfig x_encoder_config, EncoderConfig y_encoder_config, Pose current_pose=Pose());
    
    void Update(double x_encoder_raw_ticks, double y_encoder_raw_ticks, Rotation2Dd gyro_angle);

    ~FollowerOdometry();
};