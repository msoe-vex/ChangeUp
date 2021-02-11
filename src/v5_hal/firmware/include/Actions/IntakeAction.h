#pragma once

#include "api.h"
#include "Auton.h"
#include "Timer.h"
#include "DataNodes/ConveyorNode.h"
#include "DataNodes/ADIAnalogInNode.h"

class IntakeAction : public AutonAction {
private:
    ConveyorNode* m_conveyor_node;
    Timer m_timer;
    int m_voltage;
    double m_time; // Time in seconds

public:
    IntakeAction(ConveyorNode* conveyor_node, int voltage=12000, double time=0);

    void actionInit();

    actionStatus action();

    void actionEnd();
};