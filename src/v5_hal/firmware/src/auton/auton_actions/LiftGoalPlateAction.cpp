#include "auton/auton_actions/LiftGoalPlateAction.h"

LiftGoalPlateAction::LiftGoalPlateAction(IntakeNode* intake_node, bool lifted) : 
        m_intake_node(intake_node), 
        m_lifted(lifted) {
        
}

void LiftGoalPlateAction::ActionInit() {

}

AutonAction::actionStatus LiftGoalPlateAction::Action() {
    m_intake_node->liftGoalPlate((int)m_lifted);
    return END;
}

void LiftGoalPlateAction::ActionEnd() {

}