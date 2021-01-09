#include "DataNodes/ControllerNode.h"

//This constructor also calls the Node constructor from NodeManager.h
//The Node class constructor automatically adds the node so we don't have to?

//controller_id_e_t is a typedef'ed enum which can be located in the pros/misc.h file
//It can only be E_CONTROLLER_MASTER or E_CONTROLLER_PARTNER 
//CONTROLLER_MASTER and CONTROLLER_PARTNER are #define as their E_ counterparts
ControllerNode::ControllerNode(NodeManager* nodeManager, std::string handle_name, 
    pros::controller_id_e_t controller_id) : Node(nodeManager, 200) {
     m_controller = new pros::Controller(controller_id);
     m_controller_msg = new v5_hal::V5Controller;
     m_handle = new ros::NodeHandle;
     m_handle_name = handle_name;
}

void ControllerNode::initialize() {
    //Create a string to hold the handle name of the specific contorller object
    std::string controller_handle = "Controller_" + m_handle_name;

    //Create a publisher which takes the handler name and a message location 
    m_publisher = new ros::Publisher(controller_handle.c_str(), m_controller_msg);

    //Initialize the hamdler and advertise the controller message
    m_handle->initNode();
    m_handle->advertise(*m_publisher);
}

void ControllerNode::periodic() {
    m_populateControllerMsg(); //populate each value in the message file with the current value
    m_publisher->publish(m_controller_msg); //Serializes the message and queue for procesing
    m_handle->spinOnce(); //Send all queued messages
}

//Populates the V5Controller message object
void ControllerNode::m_populateControllerMsg() {
    using namespace pros::c;
    m_controller_msg->analog_left_x = controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_X);
    m_controller_msg->analog_left_y = controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y);
    m_controller_msg->analog_right_x = controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X);
    m_controller_msg->analog_right_y = controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    m_controller_msg->btn_right = controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_RIGHT);
    m_controller_msg->btn_down = controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_DOWN);
    m_controller_msg->btn_left = controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_LEFT);
    m_controller_msg->btn_up = controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_UP);
    m_controller_msg->btn_a = controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_A);
    m_controller_msg->btn_b = controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_B);
    m_controller_msg->btn_x = controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_X);
    m_controller_msg->btn_y = controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_Y);
    m_controller_msg->btn_r1 = controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_R1);
    m_controller_msg->btn_r2 = controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_R2);
    m_controller_msg->btn_l1 = controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_L1);
    m_controller_msg->btn_l2 = controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_L2);
    m_controller_msg->is_connected = controller_is_connected(pros::E_CONTROLLER_MASTER);
    m_controller_msg->battery_capacity = controller_get_battery_capacity(pros::E_CONTROLLER_MASTER);
    m_controller_msg->battery_level = controller_get_battery_level(pros::E_CONTROLLER_MASTER);
}

ControllerNode::~ControllerNode() {
    delete m_controller;
    delete m_controller_msg;
    delete m_handle;
}