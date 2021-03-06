#include "auton/auton_actions/DeployAction.h"

DeployAction::DeployAction(ConveyorNode* conveyor_node, IntakeNode* intake_node) : 
        m_conveyor_node(conveyor_node),
        m_intake_node(intake_node) {

}

void DeployAction::ActionInit() {
    m_timer.Start();

    m_conveyor_node->setBottomConveyorVoltage(MAX_MOTOR_VOLTAGE);
    m_intake_node->deployIntakes(1);
    pros::delay(1500);
}

AutonAction::actionStatus DeployAction::Action() {
    if (m_timer.Get() > 3) {
        m_conveyor_node->setBottomConveyorVoltage(0);
        return END;
    }

    return CONTINUE;
}

void DeployAction::ActionEnd() {
    m_timer.Stop();
    m_conveyor_node->setBottomConveyorVoltage(0);
}