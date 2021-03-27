#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "util/Timer.h"
#include "util/Constants.h"
#include "nodes/subsystems/IntakeNode.h"

class IntakeAction : public AutonAction {
private:
    IntakeNode* m_intake_node;
    Timer m_timer;
    int m_voltage;
    double m_time; // Time in seconds

public:
    IntakeAction(IntakeNode* intake_node, int voltage=MAX_MOTOR_VOLTAGE, double time=0);

    void ActionInit();

    actionStatus Action();

    void ActionEnd();
};