#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "util/Timer.h"
#include "nodes/subsystems/ConveyorNode.h"

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
