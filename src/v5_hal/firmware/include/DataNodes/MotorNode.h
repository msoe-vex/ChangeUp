#pragma once

#include "api.h"
#include "NodeManager.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/V5Motor.h"

class MotorNode : public Node {
private:
    pros::Motor* m_motor;
    v5_hal::V5Motor* m_motor_msg;
    ros::NodeHandle* m_handle;
    ros::Publisher* m_publisher;
    std::string m_handle_name;

    void populateMotorMsg();

public:
    MotorNode(NodeManager* nodeManager, int portNumber, std::string handleName, 
        bool reverse=false, pros::motor_gearset_e_t gearset=pros::E_MOTOR_GEARSET_18);

    void initialize();

    void periodic();

    ~MotorNode();
};
