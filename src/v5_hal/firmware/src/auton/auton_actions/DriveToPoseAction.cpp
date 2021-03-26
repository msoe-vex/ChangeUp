#include "auton/auton_actions/DriveToPoseAction.h"
#include <math.h>

DriveToPoseAction::DriveToPoseAction(IDriveNode* drive_node, OdometryNode* odom_node, Pose end_pose) :
        m_drive_node(drive_node),
        m_odom_node(odom_node), 
        m_end_pose(end_pose) {

}

void DriveToPoseAction::ActionInit() {
    m_timer.Start();
}

AutonAction::actionStatus DriveToPoseAction::Action() {
    return CONTINUE;
}

void DriveToPoseAction::ActionEnd() {
    m_drive_node->setDriveVelocity(0, 0);
}