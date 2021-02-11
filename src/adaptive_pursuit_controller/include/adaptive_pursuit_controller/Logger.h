#pragma once

#include <vector>
#include <queue>
#include <string>
#include <memory>
#include <ctime>
#include <fstream>
#include <iostream>
#include <cmath>

using namespace std;

class Logger {
public:
    enum loggingLevel {
        INFO,
        WARNING,
        ERROR
    };
    static void logInfo(ostringstream& message);
    static void logInfo(string message);
    static void logWarning(ostringstream& message);
    static void logWarning(string message);
    static void logError(ostringstream& message);
    static void logError(string message);
    static void setConsoleLoggingLevel(loggingLevel level);
private:
    static loggingLevel m_consoleLoggingLevel;
};
