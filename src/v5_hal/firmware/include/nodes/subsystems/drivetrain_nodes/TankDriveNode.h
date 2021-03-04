#pragma once

#include "nodes/NodeManager.h"
#include "api.h"
#include "nodes/actuator_nodes/MotorNode.h"
#include "nodes/sensor_nodes/ControllerNode.h"

class TankDriveNode : public Node {
private:
    pros::Controller* m_controller;

    MotorNode* m_left_front_motor;
    MotorNode* m_left_rear_motor;
    MotorNode* m_right_front_motor;
    MotorNode* m_right_rear_motor;

    std::string m_handle_name;

    const float MAX_ROBOT_SPEED = 41.9; // in/s

    void m_setLeftVoltage(int voltage);

    void m_setRightVoltage(int voltage);

    void m_setLeftVelocity(float velocity);

    void m_setRightVelocity(float velocity);

    void m_setLeftDistancePID(double distance, int max_velocity);

    void m_setRightDistancePID(double distance, int max_velocity);

public: 
    TankDriveNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, 
        MotorNode* left_front_motor, MotorNode* left_rear_motor, MotorNode* right_front_motor,
        MotorNode* right_rear_motor);

    void initialize();

    void resetEncoders();

    void setDriveVoltage(int left_voltage, int right_voltage);

    void setDriveVelocity(float left_velocity, float right_velocity);

    void setDriveDistancePID(double left_distance, double right_distance, int max_velocity);

    void teleopPeriodic();

    void autonPeriodic();

    int getRightDistancePID();

    int getLeftDistancePID();

    ~TankDriveNode();
};
