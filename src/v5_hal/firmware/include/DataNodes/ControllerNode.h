#pragma once

#include "api.h"
#include "NodeManager.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/V5Controller.h"
#include "ros_lib/std_msgs/String.h"
#include "pros/misc.h"

class ControllerNode : public Node {
private:
    pros::Controller m_controller; //Creates a new Pros Controller object
    v5_hal::V5Controller m_controller_msg; //Creates a new V5Controller message object
    std::string m_handle_name; //Creates a handle name to specify between objects
    std::string m_sub_controller_rumble_name;
    ros::Publisher* m_publisher; //Creates a new publisher object that will prepare messages for sending
    ros::Subscriber<std_msgs::String, ControllerNode>* m_rumble_controller_sub;

    void m_populateMessage();

    void m_rumbleController(const std_msgs::String& msg);

public:
    ControllerNode(NodeManager* node_manager, std::string* handle_name,
        pros::controller_id_e_t controller_id);

    void initialize();

    void periodic();

    ~ControllerNode();
};