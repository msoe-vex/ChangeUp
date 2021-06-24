#include "auton/auton_actions/FollowPathAction.h"

FollowPathAction::FollowPathAction(IDriveNode* drive_node, OdometryNode* odom_node, Path path, bool reset_pose) :
        m_drive_node(drive_node),
        m_odom_node(odom_node), 
        m_holonomic_pursuit(path),
        m_path(path),
        m_reset_pose(reset_pose) {

}

void FollowPathAction::ActionInit() {
    if (m_reset_pose) {
        m_odom_node->setCurrentPose(m_path.getPathPoints().at(0).getPose());
    }

    m_holonomic_pursuit.startPursuit();
}

AutonAction::actionStatus FollowPathAction::Action() {
    HolonomicPursuit::TargetVelocity target_velocity = m_holonomic_pursuit.getTargetVelocity(m_odom_node->getCurrentPose());

    m_drive_node->setDriveVelocity(target_velocity.linear_velocity.x(), target_velocity.linear_velocity.y(), target_velocity.rotational_velocity);

    if (m_timer.Get() == 0 && target_velocity.end_of_path) {
        m_timer.Start();
    } else if (m_timer.Get() > 0 && !target_velocity.end_of_path) {
        m_timer.Reset();
    } else if (m_timer.Get() > 0.5) {
        return END;
    }

    return CONTINUE;
}

void FollowPathAction::ActionEnd() {
    m_drive_node->setDriveVelocity(0, 0, 0);
}