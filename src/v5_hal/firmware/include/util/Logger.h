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
    enum loggingLevel {
        INFO,
        WARNING,
        ERROR
    };
    static void logInfo(string message);
    static void setConsoleLoggingLevel(loggingLevel level);
    static void giveNodeManager(NodeManager * node_manager);
private:
    static loggingLevel m_consoleLoggingLevel;
    static NodeManager * m_node_manager;
};
