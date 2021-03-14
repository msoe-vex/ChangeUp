#pragma once

#include "nodes/NodeManager.h"
#include "api.h"
#include "nodes/actuator_nodes/MotorNode.h"
#include "nodes/sensor_nodes/ControllerNode.h"
#include <algorithm>
#include <initializer_list>

class HolonomicDriveNode : public Node {
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

    void m_setLeftFrontVelocity(float voltage);

    void m_setLeftRearVelocity(float voltage);

    void m_setRightFrontVelocity(float voltage);

    void m_setRightRearVelocity(float voltage);

public: 
    struct HolonomicDriveMotorPowers
    {
       double left_front;
       double left_rear;
       double right_front;
       double right_rear;
    };

    HolonomicDriveNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, MotorNode* left_front_motor, 
        MotorNode* left_rear_motor, MotorNode* right_front_motor, MotorNode* right_rear_motor);

    void initialize();

    void resetEncoders();

    void setDriveVoltage(int left_front_voltage, int left_rear_voltage, int right_front_voltage, int right_rear_voltage);

    void setDriveVelocity(float left_front_velocity, float left_rear_velocity, float right_front_velocity, float right_rear_velocity);

    void setDriveVelocity(HolonomicDriveMotorPowers motor_powers);

    void teleopPeriodic();

    void autonPeriodic();

    int getLeftFrontPosition();

    int getLeftRearPosition();

    int getRightFrontPosition();

    int getRightRearPosition();

    ~HolonomicDriveNode();
};
