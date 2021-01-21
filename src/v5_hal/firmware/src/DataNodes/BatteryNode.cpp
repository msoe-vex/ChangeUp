#include "DataNodes/BatteryNode.h"

BatteryNode::BatteryNode (NodeManager* nodeManager, std::string* handleName)
     : Node (nodeManager, 200), m_publisher(handleName->insert(0, "battery/").c_str(), &m_battery_msg) {
    m_handle_name = handleName;
}

void BatteryNode::initialize() {
    Node::m_handle->initNode();
    Node::m_handle->advertise(m_publisher);
}

void BatteryNode::periodic() {
    populateMessage();
    m_publisher.publish(&m_battery_msg);
    Node::m_handle->spinOnce();
}

void BatteryNode::populateMessage () {
    m_battery_msg.capacity = pros::battery::get_capacity();
    m_battery_msg.current = pros::battery::get_current();
    m_battery_msg.temperature = pros::battery::get_temperature();
    m_battery_msg.voltage = pros::battery::get_temperature();
}

BatteryNode::~BatteryNode() {
    delete m_handle_name;
}