#include "auton/auton_actions/FollowPathAction.h"

FollowPathAction::FollowPathAction(IDriveNode* drive_node, OdometryNode* odom_node, Path path) :
        m_drive_node(drive_node),
        m_odom_node(odom_node), 
        m_holonomic_pursuit(path) {
}

void FollowPathAction::ActionInit() {
    m_holonomic_pursuit.startPursuit();
    
    Logger::logInfo("Starting path");
}

AutonAction::actionStatus FollowPathAction::Action() {
    HolonomicPursuit::TargetVelocity target_velocity = m_holonomic_pursuit.getTargetVelocity(m_odom_node->getCurrentPose());

    //Logger::logInfo("Desired velocity | x: " + std::to_string(target_velocity.linear_velocity.x()) + " | y: " + std::to_string(target_velocity.linear_velocity.y()));

    m_drive_node->setDriveVelocity(target_velocity.linear_velocity.x(), target_velocity.linear_velocity.y(), target_velocity.rotational_velocity);

    if (m_timer.Get() == 0 && target_velocity.end_of_path) {
        m_timer.Start();
    } else if (m_timer.Get() > 0 && !target_velocity.end_of_path) {
        m_timer.Reset();
    } else if (m_timer.Get() > 0.5) {
        Logger::logInfo("Ending path");
        return END;
    }

    return CONTINUE;
}

void FollowPathAction::ActionEnd() {
    m_drive_node->setDriveVelocity(0, 0, 0);
}