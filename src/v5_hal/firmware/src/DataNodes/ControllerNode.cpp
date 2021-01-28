#include "DataNodes/ControllerNode.h"

//This constructor also calls the Node constructor from NodeManager.h
//The Node class constructor automatically adds the node so we don't have to?

//controller_id_e_t is a typedef'ed enum which can be located in the pros/misc.h file
//It can only be E_CONTROLLER_MASTER or E_CONTROLLER_PARTNER 
//CONTROLLER_MASTER and CONTROLLER_PARTNER are #define as their E_ counterparts
ControllerNode::ControllerNode(NodeManager* node_manager, std::string handle_name,
    pros::controller_id_e_t controller_id) : Node(node_manager, 50),
    m_controller(controller_id) {
    m_handle_name = handle_name.insert(0, "joystick/");
    m_sub_controller_rumble_name = m_handle_name + "/joystickRumble";

    m_publisher = new ros::Publisher(m_handle_name.c_str(), &m_controller_msg);
    m_rumble_controller_sub = new ros::Subscriber<std_msgs::String, ControllerNode>
        (m_sub_controller_rumble_name.c_str(), &ControllerNode::m_rumbleController, this);
}

void ControllerNode::m_rumbleController(const std_msgs::String& msg) {
    std::string str = msg.data;
    if (str.length() <= 8) {
        m_controller.rumble(str.c_str());
    } else {
        // TODO Log to ROS
    }
}

void ControllerNode::initialize() {
    // Initialize the handler, and set up data to publish
    Node::m_handle->advertise(*m_publisher);
    Node::m_handle->subscribe(*m_rumble_controller_sub);
}

void ControllerNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    m_populateMessage(); //populate each value in the message file with the current value
    m_publisher->publish(&m_controller_msg); //Serializes the message and queue for procesing
}

//Populates the V5Controller message object
void ControllerNode::m_populateMessage() {
    m_controller_msg.analog_left_x = m_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    m_controller_msg.analog_left_y = m_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    m_controller_msg.analog_right_x = m_controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    m_controller_msg.analog_right_y = m_controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    m_controller_msg.btn_right = (bool)m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    m_controller_msg.btn_down = (bool)m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    m_controller_msg.btn_left = (bool)m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    m_controller_msg.btn_up = (bool)m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    m_controller_msg.btn_a = (bool)m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    m_controller_msg.btn_b = (bool)m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    m_controller_msg.btn_x = (bool)m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    m_controller_msg.btn_y = (bool)m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    m_controller_msg.btn_r1 = (bool)m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    m_controller_msg.btn_r2 = (bool)m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    m_controller_msg.btn_l1 = (bool)m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    m_controller_msg.btn_l2 = (bool)m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    m_controller_msg.is_connected = (bool)m_controller.is_connected();
    m_controller_msg.battery_capacity = m_controller.get_battery_capacity();
    m_controller_msg.battery_level = m_controller.get_battery_level();
}

ControllerNode::~ControllerNode() {
    delete m_publisher;
    delete m_rumble_controller_sub;
}