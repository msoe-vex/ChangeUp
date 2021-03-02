#include <iostream>
#include "Timer.h"

using namespace std::chrono;

double Timer::Get() {
    if(m_started && !m_stopped) {
        return getTime() - m_startTime;
    } else if(m_started && m_stopped) {
        return m_stopTime - m_startTime;
    }
    //TODO: Log -> Error, m_timer has not m_started!
    return 0;
}

void Timer::Reset() {
    m_started = false;
    m_startTime = getTime();
}

void Timer::Stop() {
    if(!m_started) {
        //TODO: Log -> Error, m_timer m_stopped without starting!
    } else {
        m_stopped = true;
        m_stopTime = getTime();
    }
}

void Timer::Start() {
    m_started = true;
    m_startTime = getTime();
}

double Timer::getTime() {
  return pros::millis() / 1000.0;
}