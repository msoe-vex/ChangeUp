#include "pathing/PathSegment.h"

PathSegment::PathSegment(Waypoint start_waypoint, Waypoint end_waypoint) : 
        m_start_waypoint(start_waypoint), 
        m_end_waypoint(end_waypoint) {
    
}

void PathSegment::initialize() {
    m_length_m = m_end_waypoint.getPosition().position(1) - m_start_waypoint.getPosition().position(1) / 
                 m_end_waypoint.getPosition().position(0) - m_start_waypoint.getPosition().position(0);

    // TODO calculate duration
}

float PathSegment::getLength() {
    return m_length_m;
}

float PathSegment::getDuration() {
    return m_duration_ms;
}
