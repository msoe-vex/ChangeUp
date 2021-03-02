#pragma once

#include "odometry/Odometry.h"

class TankOdometry : public Odometry {
public:
    TankOdometry(EncoderConfig left_encoder_config, EncoderConfig right_encoder_config, Pose current_pose=Pose());

    void Update(double left_encoder_raw_ticks, double right_encoder_raw_ticks, double track_width);
    
    void Update(double left_encoder_raw_ticks, double right_encoder_raw_ticks, Rotation2Dd gyro_angle);

    ~TankOdometry();
};