#pragma once

#include <vector>
#include <queue>
#include <string>
#include <memory>
#include <ctime>
#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>
#include "nodes/NodeManager.h"

using namespace std;

class Logger {
public:
    enum LoggingLevel {
        INFO,
        WARNING,
        ERROR
    };

    std::string m_message;
    std::string m_handle_name = "logger";

    static void initialize();
    static void logInfo(string message);
    static void setConsoleLoggingLevel(LoggingLevel level);
    static void giveNodeManager(NodeManager * node_manager);

private:
    static LoggingLevel m_console_logging_level;
    static NodeManager * m_node_manager;
};
