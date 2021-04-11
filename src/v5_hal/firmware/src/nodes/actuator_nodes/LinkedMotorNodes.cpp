#pragma once
#include "nodes/actuator_nodes/LinkedMotorNodes.h"

LinkedMotorNodes::LinkedMotorNodes(MotorNode* m1, MotorNode* m2) {
    add_motor(m1);
    add_motor(m2);
}

void LinkedMotorNodes::add_motor(MotorNode* m) {
    motor_nodes.push_back(m);
}

//Overwritten Motor_node functions
void LinkedMotorNodes::resetEncoder() {
    for (auto& m : motor_nodes) {
        m.resetEncoder();
    }
}

int LinkedMotorNodes::getPosition() {
    int average = 0;
    for (auto& m : motor_nodes) {
        average += m.getPosition();
    }
    average /= motor_nodes.size();
    return average;
}

void LinkedMotorNodes::move(int value) {
    for (auto& m : motor_nodes) {
        m.move(value);
    }
}

void LinkedMotorNodes::moveVoltage(int voltage) {
    for (auto& m : motor_nodes) {
        m.move_voltage(voltage);
    }
}

void LinkedMotorNodes::moveVelocity(float velocity) {
    for (auto& m : motor_nodes) {
        m.move_velocity(velocity);
    }
}

void LinkedMotorNodes::moveAbsolute(double position, int max_velocity) {
    for (auto& m : motor_nodes) {
        m.move_absolute(position, max_velocity);
    }
}

void LinkedMotorNodes::teleopPeriodic() {

}

void LinkedMotorNodes::autonPeriodic() {

}

