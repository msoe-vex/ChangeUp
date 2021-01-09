#include "DataNodes/BatteryNode.h"

BatteryNode::BatteryNode (NodeManager* nodeManager, std::string handleName)
     : Node (nodeManager, 200) {
    m_handle = new ros::NodeHandle();
    m_battery_msg = new v5_hal::V5Battery();
    m_handle_name = handleName;
}

void BatteryNode::initialize() {
    m_publisher = new ros::Publisher(m_handle_name.c_str(), m_battery_msg);

    m_handle->initNode();
    m_handle->advertise(*m_publisher);
}

void BatteryNode::periodic() {
    m_populateBatteryMsg();
    m_publisher->publish(m_battery_msg);
    m_handle->spinOnce();
}

void BatteryNode::m_populateBatteryMsg () {
    using namespace pros::battery;
    m_battery_msg->capacity = get_capacity();
    m_battery_msg->current = get_current();
    m_battery_msg->temperature = get_temperature();
    m_battery_msg->voltage = get_temperature();
}

BatteryNode::~BatteryNode() {
    delete m_handle;
    delete m_battery_msg;
    delete m_publisher;
}