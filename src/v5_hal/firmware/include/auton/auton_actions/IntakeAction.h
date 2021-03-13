#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "util/Timer.h"
#include "util/Constants.h"
#include "nodes/subsystems/ConveyorNode.h"

class IntakeAction : public AutonAction {
private:
    ConveyorNode* m_conveyor_node;
    Timer m_timer;
    int m_voltage;
    double m_time; // Time in seconds

public:
    IntakeAction(ConveyorNode* conveyor_node, int voltage=MAX_MOTOR_VOLTAGE, double time=0);

    void ActionInit();

    actionStatus Action();

    void ActionEnd();
};