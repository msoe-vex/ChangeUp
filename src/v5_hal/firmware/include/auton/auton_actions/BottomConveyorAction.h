#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "util/Timer.h"
#include "util/Constants.h"
#include "nodes/subsystems/ConveyorNode.h"

class BottomConveyorAction : public AutonAction {
private:
    ConveyorNode* m_conveyor_node;
    Timer m_timer;
    bool m_eject;
    int m_voltage;
    double m_time;
    int m_eject_voltage;

public:
    BottomConveyorAction(ConveyorNode* conveyor_node, bool eject, int voltage=MAX_MOTOR_VOLTAGE, double time=0.);
    
    void ActionInit();
    
    AutonAction::actionStatus Action();
    
    void ActionEnd();
};