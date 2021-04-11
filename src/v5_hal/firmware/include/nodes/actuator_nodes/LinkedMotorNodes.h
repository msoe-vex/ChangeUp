#pragma once
#include "nodes/actuator_nodes/MotorNode.h"

class LinkedMotorNodes : public MotorNode {
private:
    vector<MotorNode> motor_nodes;
public:
    LinkedMotorNodes(MotorNode* m1, MotorNode* m2);

    void add_motor(MotorNode* m);

    //void initialize();

    void resetEncoder();

    int getPosition();

    void move(int value);

    void moveVoltage(int voltage);

    void moveVelocity(float velocity);

    void moveAbsolute(double position, int max_velocity);

    void teleopPeriodic();

    void autonPeriodic();

    //~LinkedMotorNodes();
};