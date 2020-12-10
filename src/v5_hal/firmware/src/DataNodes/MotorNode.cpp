#include "DataNodes/MotorNode.h"

MotorNode::MotorNode(NodeManager* nodeManager, int portNumber, std::string handle, 
    pros::motor_gearset_e_t gearset, bool reverse):Node(nodeManager, 200) {
    m_motor = new pros::Motor(portNumber, gearset, reverse);
    m_motor_msg = new v5_hal::V5Motor();
    m_handle = new ros::NodeHandle();
}

void MotorNode::initialize() {
    // Define a topic we will publish to, and where the value will be stored
    char* data_str_buf;

    // Make a title with the port number 
    sprintf(data_str_buf, "Motor_Port%d_Pub", m_motor->get_port());


    // Create a publisher with the custom title, and message location
    m_publisher = new ros::Publisher(data_str_buf, m_motor_msg);
    
    // Initialize the handler, and set up data relating to what this node publishes
    m_handle->initNode();
    m_handle->advertise(*m_publisher);
}

void MotorNode::execute() {
    populateMotorMsg();
    m_publisher->publish(m_motor_msg);
}

void MotorNode::populateMotorMsg() {
    m_motor_msg->current_draw = m_motor->get_current_draw();
    m_motor_msg->direction = m_motor->get_direction();
    m_motor_msg->efficiency = m_motor->get_efficiency();
    m_motor_msg->is_stopped = m_motor->is_stopped();
    m_motor_msg->position = m_motor->get_position();
    m_motor_msg->port = m_motor->get_port();
    m_motor_msg->power_draw = m_motor->get_power();
    m_motor_msg->temperature = m_motor->get_temperature();
    m_motor_msg->torque = m_motor->get_torque();
    m_motor_msg->velocity = m_motor->get_actual_velocity();
    m_motor_msg->voltage = m_motor->get_voltage();
    m_motor_msg->is_over_current = m_motor->is_over_current();
    m_motor_msg->is_over_temp = m_motor->is_over_temp();
}

MotorNode::~MotorNode() {
    delete m_motor;
    delete m_motor_msg;
    delete m_handle;
}