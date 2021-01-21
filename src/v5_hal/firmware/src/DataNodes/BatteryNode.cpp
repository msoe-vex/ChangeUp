#include "DataNodes/BatteryNode.h"

BatteryNode::BatteryNode (NodeManager* node_manager, std::string* handle_name)
     : Node (node_manager, 200) {
    m_handle_name = handle_name->insert(0, "battery/");

    m_publisher = new ros::Publisher(m_handle_name.c_str(), &m_battery_msg);

    delete handle_name;
}

void BatteryNode::initialize() {
    Node::m_handle->initNode();
    Node::m_handle->advertise(*m_publisher);
}

void BatteryNode::periodic() {
    m_populateMessage();
    m_publisher->publish(&m_battery_msg);
    Node::m_handle->spinOnce();
}

void BatteryNode::m_populateMessage () {
    m_battery_msg.capacity = pros::battery::get_capacity();
    m_battery_msg.current = pros::battery::get_current();
    m_battery_msg.temperature = pros::battery::get_temperature();
    m_battery_msg.voltage = pros::battery::get_temperature();
}

BatteryNode::~BatteryNode() {
    delete m_publisher;
}