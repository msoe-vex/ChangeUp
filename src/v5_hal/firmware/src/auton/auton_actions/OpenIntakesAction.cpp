#include "auton/auton_actions/OpenIntakesAction.h"

OpenIntakesAction::OpenIntakesAction(IntakeNode* intake_node, bool open) : 
        m_intake_node(intake_node), 
        m_open(open) {
        
}

void OpenIntakesAction::ActionInit() {

}

AutonAction::actionStatus OpenIntakesAction::Action() {
    m_intake_node->openIntakes((int)m_open);
    return END;
}

void OpenIntakesAction::ActionEnd() {

}