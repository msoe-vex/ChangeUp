#pragma once

#include "api.h"
#include "NodeManager.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/V5Controller.h"
#include "pros/misc.h"

class ControllerNode : public Node {
private:
    pros::Controller* m_controller; //Creates a new Pros Controller object
    v5_hal::V5Controller* m_controller_msg; //Creates a new V5Controller message object
    ros::NodeHandle* m_handle; //Creates a new NodeHandle object with methods to handle basic node functions
    ros::Publisher* m_publisher; //Creates a new publisher object that will prepare messages for sending
    std::string m_handle_name; //Creates a handle name to specify between objects

    void m_populateControllerMsg();

public:
    ControllerNode(NodeManager* nodeManager, std::string handle_name, 
        pros::controller_id_e_t controller_id);

    void initialize();

    void periodic();

    ~ControllerNode ();
};