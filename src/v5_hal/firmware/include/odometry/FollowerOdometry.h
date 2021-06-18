#pragma once

#include "odometry/Odometry.h"
#include "util/Constants.h"
#include "util/Logger.h"

#include <math.h>

class FollowerOdometry : public Odometry {
private:
    bool m_previously_positive = true;

public:
    FollowerOdometry(EncoderConfig x_encoder_config, EncoderConfig y_encoder_config, Pose current_pose=Pose());
    
    void Update(double x_encoder_raw_ticks, double y_encoder_raw_ticks, Rotation2Dd gyro_angle);

    ~FollowerOdometry();
};