#include "NodeManager.h"

NodeManager::NodeManager(long unsigned int (*getMilliseconds)(void)) { 
    m_getMillis = getMilliseconds; 
}

void NodeManager::addNode(Node* node, int intervalMilliseconds) {
    NodeManager::NodeStructure nodeStructure = {node, intervalMilliseconds, 0};
    m_nodeStructures.push_back(nodeStructure);
}

void NodeManager::initialize() {
    for (auto nodeStructure : m_nodeStructures) {
        nodeStructure.node->initialize();
    }
}

void NodeManager::execute() {
    for (auto nodeStructure : m_nodeStructures) {
        auto currentTime = m_getMillis();
        if (currentTime - nodeStructure.lastExecutedMillis >=
            nodeStructure.triggerMillis) {
            nodeStructure.node->periodic();
            nodeStructure.lastExecutedMillis = currentTime;
        }
    }
}

NodeManager::~NodeManager() { m_nodeStructures.clear(); }