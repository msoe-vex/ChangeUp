#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "util/Timer.h"
#include "nodes/subsystems/ConveyorNode.h"

class ScoreSingleBallAction : public AutonAction {
private:
    ConveyorNode* m_conveyor_node;
    bool m_wait_for_next_ball;
    bool m_ball_scored = false;
    Timer m_timer;

public:
    ScoreSingleBallAction(ConveyorNode* conveyor_node, bool wait_for_next_ball=false);

    void ActionInit();

    actionStatus Action();

    void ActionEnd();
};