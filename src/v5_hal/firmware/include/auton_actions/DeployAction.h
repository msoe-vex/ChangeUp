#pragma once

#include "api.h"
#include "Auton.h"
#include "Timer.h"
#include "data_nodes/ConveyorNode.h"

class DeployAction : public AutonAction {
private:
    ConveyorNode* m_conveyor_node;
    Timer m_timer;
public:
    DeployAction(ConveyorNode* conveyor_node);

    void ActionInit();
    actionStatus Action();
    void ActionEnd();
};
