#include "NodeManager.h"

NodeManager::NodeManager(uint32_t(*get_milliseconds)(void)) {
    m_get_millis = get_milliseconds;
    m_handle = new ros::NodeHandle();
}

ros::NodeHandle* NodeManager::addNode(Node* node,
    uint32_t interval_milliseconds) {
    NodeManager::NodeStructure node_structure = { node, interval_milliseconds, 0 };
    m_node_structures.push_back(node_structure);

    return m_handle;
}

void NodeManager::initialize() {
    m_handle->initNode();
    for (auto node_structure : m_node_structures) {
        node_structure.node->initialize();
    }
}

void NodeManager::execute() {
    switch(m_handle->spinOnce()) {
        case ros::SPIN_ERR:
            m_handle->logerror("spin once encountered error");
            break;
        case ros::SPIN_TIMEOUT:
            m_handle->logerror("spin once timed out!");
            break;
        default:
            break;
    }
    
    for (auto& node_structure : m_node_structures) {
        auto current_time = m_get_millis();
        if (current_time - node_structure.last_executed_millis >=
            node_structure.trigger_millis) {
            node_structure.node->periodic();
            node_structure.last_executed_millis = current_time;
        }
    }
    pros::c::delay(m_delay_time_millis);
}

NodeManager::~NodeManager() { m_node_structures.clear(); }