#include "util/Logger.h"

Logger::LoggingLevel Logger::m_console_logging_level;
NodeManager* Logger::m_node_manager = nullptr;

void Logger::initialize() {
    m_publisher = new ros::Publisher(m_handle_name.c_str(), &m_message);
    Node::m_handle->advertise(*m_publisher);
}

void Logger::giveNodeManager(NodeManager* node_manager) {
    m_node_manager = node_manager;
}

void Logger::logInfo(string message) {
    if(m_node_manager != nullptr) {
        m_message = message;
        m_publisher->publish(&m_message);
    }
}

void Logger::setConsoleLoggingLevel(Logger::LoggingLevel level) {
    m_console_logging_level = level;
}
