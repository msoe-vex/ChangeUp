#pragma once

#include "NodeManager.h"
#include "api.h"
#include "DataNodes/MotorNode.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/Int8.h"
#include "ros_lib/std_msgs/Empty.h"

class TankDriveNode : public Node {
private:
    MotorNode* m_left_front_motor;
    MotorNode* m_left_rear_motor;
    MotorNode* m_right_front_motor;
    MotorNode* m_right_rear_motor;

    std::string m_handle_name;

    void m_resetEncoders();

    void m_setLeftVoltage(int voltage);

    void m_setRightVoltage(int voltage);

    void m_setLeftDistancePID(double distance, int max_velocity);

    void m_setRightDistancePID(double distance, int max_velocity);

public: 
    TankDriveNode(NodeManager* node_manager, std::string handle_name, MotorNode* left_front_motor, 
        MotorNode* left_rear_motor, MotorNode* right_front_motor, MotorNode* right_rear_motor);

    void initialize();

    void setDriveVoltage(int left_voltage, int right_voltage);

    void setDriveDistancePID(double left_distance, double right_distance, int max_velocity);

    void periodic();

    ~TankDriveNode();
};
