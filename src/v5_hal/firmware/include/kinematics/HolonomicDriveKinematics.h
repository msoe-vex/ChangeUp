#pragma once

#include "kinematics/IDriveKinematics.h"
#include "util/Constants.h"
#include <algorithm>
#include <initializer_list>

class HolonomicDriveKinematics : public IDriveKinematics {
private:
    float m_left_front_previous_dist;
    float m_left_rear_previous_dist;
    float m_right_front_previous_dist;
    float m_right_rear_previous_dist;

    float m_getWheelTicksComponent(float wheel_ticks);

public:
    HolonomicDriveKinematics(EncoderConfig encoder_config, Pose current_pose=Pose());

    /**
     * This function takes in encoder values of all motors, and uses them to update
     * the position of the robot based on the change of position over time
     *  
     * @param encoder_vals struct holding encoder values for all motors
     */
    void updateForwardKinematics(IDriveNode::FourMotorDriveEncoderVals encoder_vals);

    /**
     * This function takes in movements of a drivetrain in the x, y, and theta axes.
     * The inputs of this function must be of the same units (m/s, volts, etc.) and 
     * the function will return the proportion of the supplied maximum to send to each
     * 
     * @param x float representing the x-movement of the drivetrain
     * @param y float representing the y-movement of the drivetrain
     * @param theta float representing the rotational movement of the drivetrain
     * @returns struct containing percentage of the supplied maximum to send to each motor
     */
    FourMotorPercentages inverseKinematics(float x, float y, float theta, float max);

    ~HolonomicDriveKinematics();
};