#pragma once

#include "api.h"
#include "nodes/subsystems/drivetrain_nodes/IDriveNode.h"
#include "math/Pose.h"
#include "util/Encoders.h"

class IDriveKinematics { 
protected:
    Rotation2Dd m_initial_angle;
    Pose m_pose = Pose(Vector2d(0, 0), Rotation2Dd());
    bool m_pose_reset = true;
    float m_ticks_to_distance_m;

public:
    struct FourMotorPercentages {
        float left_front_percent;
        float left_rear_percent;
        float right_front_percent;
        float right_rear_percent;
    };

    IDriveKinematics(EncoderConfig encoder_config, Pose current_pose=Pose());

    /**
     * This function returns the current pose of the robot
     * 
     * @returns Pose object containing the position and rotation of the robot
     */
    Pose getPose();

    /**
     * This function sets the current pose of the robot
     * 
     * @param current_pose pose to set the robot to
     */
    void setCurrentPose(Pose current_pose); 

    /**
     * This function takes in encoder values of all motors, and uses them to update
     * the position of the robot based on the change of position over time
     *  
     * @param encoder_vals struct holding encoder values for all motors
     */
    virtual void updateForwardKinematics(IDriveNode::FourMotorDriveEncoderVals encoder_vals) = 0;

    /**
     * This function takes in movements of a drivetrain in the x, y, and theta axes.
     * The inputs of this function must be of the same units (m/s, volts, etc.) and 
     * the function will return the proportion of the supplied maximum to send to each
     * of four motors
     * 
     * @param x float representing the x-movement of the drivetrain
     * @param y float representing the y-movement of the drivetrain
     * @param theta float representing the rotational movement of the drivetrain
     * @returns struct containing percentage of the supplied maximum to send to each motor
     */
    virtual FourMotorPercentages inverseKinematics(float x, float y, float theta, float max) = 0;

    ~IDriveKinematics();
};