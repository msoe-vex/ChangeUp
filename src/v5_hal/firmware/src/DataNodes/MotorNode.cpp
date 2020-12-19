#include "DataNodes/MotorNode.h"

// By default, this constructor calls the constructor for the Node object in NodeManager.h
MotorNode::MotorNode(NodeManager* nodeManager, int portNumber, std::string handleName, 
    bool reverse, pros::motor_gearset_e_t gearset):Node(nodeManager, 200) {
    m_motor = new pros::Motor(portNumber, gearset, reverse);
    m_motor_msg = new v5_hal::V5Motor();
    m_handle = new ros::NodeHandle();
    m_handle_name = handleName;
}

void MotorNode::initialize() {
    // Define a string handle for the motor
    std::string motor_handle = "Motor_" + m_handle_name;

    // Create a publisher with the custom title, and message location
    m_publisher = new ros::Publisher(motor_handle.c_str(), m_motor_msg);
    
    // Initialize the handler, and set up data relating to what this node publishes
    m_handle->initNode();
    m_handle->advertise(*m_publisher);
}

void MotorNode::periodic() {
    populateMotorMsg();
    m_publisher->publish(m_motor_msg);
    m_handle->spinOnce();
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