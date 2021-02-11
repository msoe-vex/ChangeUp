#include "Actions/DriveAction.h"
#include "Timer.h"

DriveAction::DriveAction(tank_drive_node) : m_tank_drive_node(tank_drive_node) {

}

void DriveAction::actionInit() {
    m_timer.Start();
}

AutonAction::actionStatus DriveAction::action() {
    

    if(m_timer.Get() > 1) {
        return END;
    } else {
        return CONTINUE;
    }
}

void DriveAction::actionEnd() {
    m_timer.Stop();
    m_conveyor_node->setIntakeVoltage(0);
    m_conveyor_node->setBottomConveyorVoltage(0);
}