#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/V5Controller.h"
#include "pros/misc.h"

class ControllerNode : public Node {
private:
    pros::Controller m_controller; //Creates a new Pros Controller object
    v5_hal::V5Controller m_controller_msg; //Creates a new V5Controller message object
    ros::Publisher m_publisher; //Creates a new publisher object that will prepare messages for sending
    std::string* m_handle_name; //Creates a handle name to specify between objects

    void populateMessage();

public:
    ControllerNode(NodeManager* nodeManager, std::string* handle_name, 
        pros::controller_id_e_t controller_id);

    void initialize();

    void periodic();

    ~ControllerNode ();
};