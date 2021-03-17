#include "kinematics/IDriveKinematics.h"

IDriveKinematics::IDriveKinematics(EncoderConfig encoder_config, Pose current_pose) :
        m_ticks_to_distance_m((encoder_config.wheel_diameter * M_PI) / 
            encoder_config.ticks_per_wheel_revolution) {
        setCurrentPose(current_pose);
}

Pose IDriveKinematics::getPose() {
    return m_pose;
}

void IDriveKinematics::setCurrentPose(Pose current_pose) {
    m_pose = current_pose;
}