#include "DataNodes/MotorNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
MotorNode::MotorNode(NodeManager* nodeManager, int portNumber,
    std::string* handleName, bool reverse,
    pros::motor_gearset_e_t gearset) : Node(nodeManager, 20),
    m_motor(portNumber, gearset, reverse),
    m_publisher(handleName->insert(0, "motor_").c_str(), &m_motor_msg),
    m_moveMotorVoltageSub("cmd_moveMotorVoltage", &MotorNode::moveMotorVoltage, this) {
    m_handle_name = handleName;
}

void MotorNode::moveMotorVoltage(const v5_hal::V5Controller& msg) {
    float speed = (msg.analog_left_y / 127) * 12000.0; // Need to be a float so you don't truncate to 0 in the first part
    m_motor.move_voltage((int)speed);
}

void MotorNode::initialize() {
    // Initialize the handler, and set up data to publish
    Node::m_handle->initNode();
    Node::m_handle->advertise(m_publisher);

    Node::m_handle->subscribe(m_moveMotorVoltageSub);
}

void MotorNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    populateMessage();
    m_publisher.publish(&m_motor_msg);
    Node::m_handle->spinOnce();
}

void MotorNode::populateMessage() {
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


MotorNode::~MotorNode() { delete m_handle_name; }