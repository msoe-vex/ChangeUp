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

    void populateMotorMsg();

public:
    MotorNode(NodeManager* nodeManager, int portNumber, std::string handle, 
        pros::motor_gearset_e_t gearset=pros::E_MOTOR_GEARSET_18, bool reverse=false);

    void initialize();

    void execute();

    ~MotorNode();
};
