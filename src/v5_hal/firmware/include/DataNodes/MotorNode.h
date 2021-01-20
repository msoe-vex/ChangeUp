#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/V5Motor.h"
#include "ros_lib/v5_hal/V5Controller.h"

class MotorNode : public Node {
private:
    pros::Motor m_motor;
    v5_hal::V5Motor m_motor_msg;
    ros::Publisher m_publisher;
    std::string* m_handle_name;
    ros::Subscriber<v5_hal::V5Controller, MotorNode> m_moveMotorVoltageSub;

    void populateMessage();

    void moveMotorVoltage(const v5_hal::V5Controller& msg);

public:
    MotorNode(NodeManager* nodeManager, int portNumber, std::string* handleName,
        bool reverse = false,
        pros::motor_gearset_e_t gearset = pros::E_MOTOR_GEARSET_18);

    void initialize();

    void periodic();

    ~MotorNode();
};
