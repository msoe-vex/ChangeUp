#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "util/Timer.h"
#include "nodes/subsystems/ConveyorNode.h"
#include "nodes/subsystems/IntakeNode.h"

class DeployAction : public AutonAction {
private:
    ConveyorNode* m_conveyor_node;
    IntakeNode* m_intake_node;
    Timer m_timer;

public:
    DeployAction(ConveyorNode* conveyor_node, IntakeNode* intake_node);

    void ActionInit();

    actionStatus Action();
    
    void ActionEnd();
};
