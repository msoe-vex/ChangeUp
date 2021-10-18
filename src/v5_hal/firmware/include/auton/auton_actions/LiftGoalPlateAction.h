#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "util/Timer.h"
#include "nodes/subsystems/IntakeNode.h"

class LiftGoalPlateAction : public AutonAction {
private:
    IntakeNode* m_intake_node;
    bool m_lifted;

public:
    LiftGoalPlateAction(IntakeNode* intake_node, bool lifted=true);
    
    void ActionInit();
    
    AutonAction::actionStatus Action();
    
    void ActionEnd();
};