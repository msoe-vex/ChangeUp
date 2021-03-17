#pragma once

#include "nodes/NodeManager.h"
#include "api.h"
#include "nodes/subsystems/drivetrain_nodes/IDriveNode.h"
#include "nodes/actuator_nodes/MotorNode.h"
#include "nodes/sensor_nodes/ControllerNode.h"
#include "kinematics/HolonomicDriveKinematics.h"
#include <algorithm>
#include <initializer_list>

class HolonomicDriveNode : public IDriveNode {
public: 
    struct HolonomicMotors {
        MotorNode* left_front_motor;
        MotorNode* left_rear_motor;
        MotorNode* right_front_motor;
        MotorNode* right_rear_motor;
    };

    HolonomicDriveNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, 
        HolonomicMotors motors, HolonomicDriveKinematics kinematics);

    void initialize();

    void resetEncoders();

    IDriveNode::FourMotorDriveEncoderVals getIntegratedEncoderVals();

    void setDriveVoltage(int x_voltage, int theta_voltage);

    void setDriveVoltage(int x_voltage, int y_voltage, int theta_voltage);

    void setDriveVelocity(float x_velocity, float theta_velocity);

    void setDriveVelocity(float x_velocity, float y_velocity, float theta_velocity);

    void teleopPeriodic();

    void autonPeriodic();

    ~HolonomicDriveNode();

private:
    pros::Controller* m_controller;

    std::string m_handle_name;

    HolonomicMotors m_motors;

    HolonomicDriveKinematics m_kinematics;

    void m_setLeftFrontVoltage(int voltage);

    void m_setLeftRearVoltage(int voltage);

    void m_setRightFrontVoltage(int voltage);

    void m_setRightRearVoltage(int voltage);

    void m_setLeftFrontVelocity(float velocity);

    void m_setLeftRearVelocity(float velocity);

    void m_setRightFrontVelocity(float velocity);

    void m_setRightRearVelocity(float velocity);
};
