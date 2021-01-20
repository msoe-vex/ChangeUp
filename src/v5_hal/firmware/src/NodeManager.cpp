#include "NodeManager.h"

NodeManager::NodeManager(uint32_t(*getMilliseconds)(void)) {
    m_getMillis = getMilliseconds;
    m_handle = new ros::NodeHandle();
}

ros::NodeHandle* NodeManager::addNode(Node* node,
    uint32_t intervalMilliseconds) {
    NodeManager::NodeStructure nodeStructure = { node, intervalMilliseconds, 0 };
    m_nodeStructures.push_back(nodeStructure);

    return m_handle;
}

void NodeManager::initialize() {
    for (auto nodeStructure : m_nodeStructures) {
        nodeStructure.node->initialize();
    }
}

void NodeManager::execute() {
    for (auto& nodeStructure : m_nodeStructures) {
        auto currentTime = m_getMillis();
        if (currentTime - nodeStructure.lastExecutedMillis >=
            nodeStructure.triggerMillis) {
            nodeStructure.node->periodic();
            nodeStructure.lastExecutedMillis = currentTime;
        }
    }
    pros::c::delay(m_delayTimeMillis);
}

NodeManager::~NodeManager() { m_nodeStructures.clear(); }