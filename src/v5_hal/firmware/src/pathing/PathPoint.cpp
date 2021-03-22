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

    Vector2d acceleration = (other.getLinearVelocity() + getLinearVelocity()) / deltaTime;
    Vector2d position = 0.5 * acceleration * (t * t) + getLinearVelocity() * t + getPose().vector;
    Vector2d velocity = acceleration * t + getLinearVelocity();

    Rotation2Dd alpha = (other.getRotationalVelocity() - getRotationalVelocity()) / deltaTime;
    Rotation2Dd theta = 0.5 * alpha * (t * t) + getRotationalVelocity() * t + getPose().rotation;
    Rotation2Dd omega = alpha * t + getRotationalVelocity();

    return PathPoint(time, Pose(position, theta), velocity, omega);
}