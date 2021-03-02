#include "Logger.h"

Logger::loggingLevel Logger::m_consoleLoggingLevel;


void Logger::logInfo(ostringstream& message) {
	//logInfo(message.str());
}

void Logger::logInfo(string message) {
    if(m_consoleLoggingLevel <= INFO) {
        cout << "INFO: " << message << "\n";
    }
}

void Logger::logWarning(ostringstream& message) {
//	logWarning(message.str());
}

void Logger::logWarning(string message) {
    if(m_consoleLoggingLevel <= WARNING) {
        cout << "WARNING: " << message << "\n";
    }
}

void Logger::logError(ostringstream& message) {
//	logError(message.str());
}

void Logger::logError(string message) {
    if(m_consoleLoggingLevel <= WARNING) {
        cout << "ERROR: " << message << "\n";
    }
}

void Logger::setConsoleLoggingLevel(Logger::loggingLevel level) {
    m_consoleLoggingLevel = level;
}
