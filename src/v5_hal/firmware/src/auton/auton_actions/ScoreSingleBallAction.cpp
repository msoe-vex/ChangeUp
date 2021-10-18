#include "auton/auton_actions/ScoreSingleBallAction.h"

ScoreSingleBallAction::ScoreSingleBallAction(ConveyorNode* conveyor_node, bool wait_for_next_ball) : 
        m_conveyor_node(conveyor_node), 
        m_wait_for_next_ball(wait_for_next_ball) {
    
}

void ScoreSingleBallAction::ActionInit() {
    m_timer.Reset();
}

AutonAction::actionStatus ScoreSingleBallAction::Action() {
    if (!m_ball_scored && m_conveyor_node->isBallInTopPosition()) {
        // Desired ball is at top of the conveyor
        m_conveyor_node->setConveyorState(ConveyorNode::SCORING);
        m_timer.Reset();
    } else if (!m_ball_scored && !m_conveyor_node->isBallInTopPosition()) {
        if (!m_timer.isStarted()) {
            m_timer.Start();
        } else if (m_timer.Get() > 0.3) {
            // Desired ball has just left the conveyor
            m_conveyor_node->setConveyorState(ConveyorNode::HOLDING);
            m_ball_scored = true;

            if (!m_wait_for_next_ball) {
                return END;
            }
        }
    } else if (m_ball_scored && m_conveyor_node->isBallInTopPosition()) {
        return END;
    }

    return CONTINUE;
}

void ScoreSingleBallAction::ActionEnd() {
    
}