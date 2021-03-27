#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "util/Timer.h"
#include "nodes/subsystems/IntakeNode.h"

class OpenIntakesAction : public AutonAction {
private:
    IntakeNode* m_intake_node;
    bool m_open;

public:
    OpenIntakesAction(IntakeNode* intake_node, bool open=true);
    
    void ActionInit();
    
    AutonAction::actionStatus Action();
    
    void ActionEnd();
};