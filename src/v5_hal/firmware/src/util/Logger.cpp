#include "util/Logger.h"

Logger::loggingLevel Logger::m_consoleLoggingLevel;
NodeManager * Logger::m_node_manager = nullptr;

void Logger::giveNodeManager(NodeManager * node_manager) {
    m_node_manager = node_manager;
}

void Logger::logInfo(string message) {
    if(m_node_manager != nullptr) {
        string msg = message;
        m_node_manager->m_handle->logwarn(msg.c_str());
    }
}

void Logger::setConsoleLoggingLevel(Logger::loggingLevel level) {
    m_consoleLoggingLevel = level;
}
