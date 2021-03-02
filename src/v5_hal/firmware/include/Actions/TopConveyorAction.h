#pragma once

#include "api.h"
#include "Auton.h"
#include "Timer.h"
#include "DataNodes/ConveyorNode.h"

class TopConveyorAction : public AutonAction {
private:
    ConveyorNode* m_conveyor_node;
    Timer m_timer;
    ConveyorNode::ConveyorState m_conveyor_state;
    int m_voltage;
    double m_time; // Time in seconds

public:
    TopConveyorAction(ConveyorNode* conveyor_node, ConveyorNode::ConveyorState conveyor_state, 
        double time=0);

    void ActionInit();

    actionStatus Action();

    void ActionEnd();
};