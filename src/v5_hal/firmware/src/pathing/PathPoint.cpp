#include "pathing/PathPoint.h"

PathPoint::PathPoint(float time, Pose pose, Vector2d linear_velocity, float rotational_velocity) {
    m_time = time;
    m_pose = pose;
}

float PathPoint::getTime() {
    return m_time;
}

Pose PathPoint::getPose() {
    return m_pose;
}

Vector2d PathPoint::getLinearVelocity() {
    return m_linear_velocity;
}

float PathPoint::getRotationalVelocity() {
    return m_rotational_velocity;
}

PathPoint PathPoint::interpolateTo(PathPoint other, float time) {
    float t = time - getTime();
    float deltaTime = other.getTime() - getTime();

    Vector2d acceleration = deltaTime == 0. ? Vector2d(0., 0.) : ((other.getLinearVelocity() + getLinearVelocity()) / deltaTime);
    Vector2d position = 0.5 * acceleration * (t * t) + getLinearVelocity() * t + getPose().position;
    Vector2d velocity = acceleration * t + getLinearVelocity();

    Logger::logInfo("Time: " + std::to_string(time) + " | position: " + std::to_string(position.x()) + " " + std::to_string(position.y()) + 
        " | velocity: " + std::to_string(velocity.x()) + " " + std::to_string(velocity.y()));

    float alpha = deltaTime == 0. ? 0. : (other.getRotationalVelocity() - getRotationalVelocity()) / deltaTime;
    Rotation2Dd theta = Rotation2Dd(0.5 * alpha * (t * t) + getRotationalVelocity() * t) * getPose().angle;
    float omega = alpha * t + getRotationalVelocity();

    return PathPoint(time, Pose(position, theta), velocity, omega);
}