#pragma once

#include "api.h"
#include "Auton.h"
#include "Timer.h"
#include "DataNodes/ConveyorNode.h"

class DeployAction : public AutonAction {
private:
    ConveyorNode* m_conveyor_node;
    Timer m_timer;
public:
    DeployAction(ConveyorNode* conveyor_node);

    void actionInit();
    actionStatus action();
    void actionEnd();
};