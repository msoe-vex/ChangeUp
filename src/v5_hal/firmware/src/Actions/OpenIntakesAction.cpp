#include "Actions/OpenIntakesAction.h"

OpenIntakesAction::OpenIntakesAction(ConveyorNode* conveyor_node, bool open) : 
        m_conveyor_node(conveyor_node), m_open(open) {
        
}

void OpenIntakesAction::ActionInit() {

}

AutonAction::actionStatus OpenIntakesAction::Action() {
    return END;
}

void OpenIntakesAction::ActionEnd() {

}