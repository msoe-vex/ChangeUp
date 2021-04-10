#pragma once
#include "nodes/MotorNode.h"

class LinkedMotorNodes : public MotorNode {
private:
    vector<MotorNode> motor_nodes;
public:
    LinkedMotorNodes(MotorNode* m1, MotorNode* m2);

    void add_motor(MotorNode* m);
};