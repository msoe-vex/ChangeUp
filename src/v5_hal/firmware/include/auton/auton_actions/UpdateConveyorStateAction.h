#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "util/Timer.h"
#include "nodes/subsystems/ConveyorNode.h"

class UpdateConveyorStateAction : public AutonAction {
private:
    ConveyorNode* m_conveyor_node;
    Timer m_timer;
    ConveyorNode::ConveyorState m_conveyor_state;
    double m_time; // Time in seconds

public:
    UpdateConveyorStateAction(ConveyorNode* conveyor_node, ConveyorNode::ConveyorState conveyor_state, 
        double time=0);

    void ActionInit();

    actionStatus Action();

    void ActionEnd();
};