#include "pathing/Waypoint.h"

Waypoint::Waypoint(Pose position, Vector2d velocity) :
        m_position(position), 
        m_velocity(velocity) {
    
}

Pose Waypoint::getPosition() {
    return m_position;
}

Vector2d Waypoint::getVelocity() {
    return m_velocity;
}