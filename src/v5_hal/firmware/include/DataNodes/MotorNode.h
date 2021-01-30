#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/V5Motor.h"
#include "ros_lib/std_msgs/Int8.h"

class MotorNode : public Node {
private:
    pros::Motor m_motor;
    v5_hal::V5Motor m_motor_msg;
    std::string m_handle_name;
    std::string m_sub_move_motor_voltage_name;
    ros::Publisher* m_publisher;
    ros::Subscriber<std_msgs::Int8, MotorNode>* m_move_motor_voltage_sub;

    void m_populateMessage();

    void m_moveMotorVoltage(const std_msgs::Int8& msg);

public:
    MotorNode(NodeManager* node_manager, int port_number, std::string handle_name,
        bool reverse=false, pros::motor_gearset_e_t gearset=pros::E_MOTOR_GEARSET_18);

    void initialize();

    int getPosition();

    void moveVoltage(int voltage);

    void periodic();

    ~MotorNode();
};
