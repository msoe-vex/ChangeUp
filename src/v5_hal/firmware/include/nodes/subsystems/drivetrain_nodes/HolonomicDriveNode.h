#pragma once

#include "nodes/NodeManager.h"
#include "api.h"
#include "nodes/subsystems/drivetrain_nodes/IDriveNode.h"
#include "nodes/actuator_nodes/MotorNode.h"
#include "nodes/sensor_nodes/ControllerNode.h"
#include <algorithm>
#include <initializer_list>

class HolonomicDriveNode : public IDriveNode {
private:
    pros::Controller* m_controller;

    MotorNode* m_left_front_motor;
    MotorNode* m_left_rear_motor;
    MotorNode* m_right_front_motor;
    MotorNode* m_right_rear_motor;

    std::string m_handle_name;

    void m_setLeftFrontVoltage(int voltage);

    void m_setLeftRearVoltage(int voltage);

    void m_setRightFrontVoltage(int voltage);

    void m_setRightRearVoltage(int voltage);

    void m_setLeftFrontVelocity(float velocity);

    void m_setLeftRearVelocity(float velocity);

    void m_setRightFrontVelocity(float velocity);

    void m_setRightRearVelocity(float velocity);

public: 
    HolonomicDriveNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, MotorNode* left_front_motor, 
        MotorNode* left_rear_motor, MotorNode* right_front_motor, MotorNode* right_rear_motor);

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
};
