#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/V5Motor.h"

class MotorNode : public Node {
   private:
    pros::Motor m_motor;
    v5_hal::V5Motor m_motor_msg;
    ros::Publisher m_publisher;
    std::string* m_handle_name;

    void populateMessage();

   public:
    MotorNode(NodeManager* nodeManager, int portNumber, std::string* handleName,
              bool reverse = false,
              pros::motor_gearset_e_t gearset = pros::E_MOTOR_GEARSET_18);

    void initialize();

    void periodic();

    ~MotorNode();
};
