#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "util/Timer.h"
#include "nodes/subsystems/ConveyorNode.h"

class OpenIntakesAction : public AutonAction {
private:
    ConveyorNode* m_conveyor_node;
    bool m_open;

public:
    OpenIntakesAction(ConveyorNode* conveyor_node, bool open=true);
    
    void ActionInit();
    
    AutonAction::actionStatus Action();
    
    void ActionEnd();
};