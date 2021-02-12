#pragma once

#include "api.h"
#include "Auton.h"
#include "Timer.h"
#include "DataNodes/ConveyorNode.h"

class TopConveyorAction : public AutonAction {
private:
    ConveyorNode* m_conveyor_node;
    Timer m_timer;
    bool m_store_balls;
    int m_voltage;
    double m_time; // Time in seconds

public:
    TopConveyorAction(ConveyorNode* conveyor_node, bool store_balls, int voltage=12000, double time=0);

    void actionInit();

    actionStatus action();

    void actionEnd();
};