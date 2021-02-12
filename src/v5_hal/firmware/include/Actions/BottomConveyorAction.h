#pragma once

#include "api.h"
#include "Auton.h"
#include "Timer.h"
#include "DataNodes/ConveyorNode.h"

class BottomConveyorAction : public AutonAction {
private:
    ConveyorNode* m_conveyor_node;
    Timer m_timer;
    bool m_eject;
    int m_voltage;
    double m_time;
    int m_eject_voltage;

public:
    BottomConveyorAction(ConveyorNode* conveyor_node, bool eject, double time, int voltage=12000);
    
    void actionInit();
    
    AutonAction::actionStatus action();
    
    void actionEnd();
};