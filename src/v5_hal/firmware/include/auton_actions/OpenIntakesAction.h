#pragma once

#include "api.h"
#include "Auton.h"
#include "Timer.h"
#include "data_nodes/ConveyorNode.h"

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