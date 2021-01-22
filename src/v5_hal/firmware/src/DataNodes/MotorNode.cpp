#include "DataNodes/MotorNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
MotorNode::MotorNode(NodeManager* node_manager, int port_number,
    std::string handle_name, bool reverse,
    pros::motor_gearset_e_t gearset) : Node(node_manager, 20),
    m_motor(port_number, gearset, reverse) {
    m_handle_name = handle_name.insert(0, "motor/");
    m_sub_move_motor_voltage_name = m_handle_name + "/moveMotorVoltage";

    m_publisher = new ros::Publisher(m_handle_name.c_str(), &m_motor_msg);
    m_move_motor_voltage_sub = new ros::Subscriber<std_msgs::Int8, MotorNode>
        (m_sub_move_motor_voltage_name.c_str(), &MotorNode::m_moveMotorVoltage, this);
}

void MotorNode::m_moveMotorVoltage(const std_msgs::Int8& msg) {
    float voltage = (msg.data / 127.0) * 12000.0;
    m_motor.move_voltage((int)voltage);
}

void MotorNode::initialize() {
    // Initialize the handler, and set up data to publish
    Node::m_handle->initNode();
    Node::m_handle->advertise(*m_publisher);
    Node::m_handle->subscribe(*m_move_motor_voltage_sub);
}

void MotorNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    m_populateMessage();
    m_publisher->publish(&m_motor_msg);
    Node::m_handle->spinOnce();
}

void MotorNode::m_populateMessage() {
    m_motor_msg.current_draw = m_motor.get_current_draw();
    m_motor_msg.direction = m_motor.get_direction();
    m_motor_msg.efficiency = m_motor.get_efficiency();
    m_motor_msg.is_stopped = m_motor.is_stopped();
    m_motor_msg.position = m_motor.get_position();
    m_motor_msg.port = m_motor.get_port();
    m_motor_msg.power_draw = m_motor.get_power();
    m_motor_msg.temperature = m_motor.get_temperature();
    m_motor_msg.torque = m_motor.get_torque();
    m_motor_msg.velocity = m_motor.get_actual_velocity();
    m_motor_msg.voltage = m_motor.get_voltage();
    m_motor_msg.is_over_current = m_motor.is_over_current();
    m_motor_msg.is_over_temp = m_motor.is_over_temp();
}


MotorNode::~MotorNode() {
    delete m_publisher;
    delete m_move_motor_voltage_sub;
}