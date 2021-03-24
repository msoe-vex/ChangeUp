#include "auton/auton_actions/FollowPathAction.h"

// FollowPathAction::FollowPathAction(TankDriveNode* tank_drive, OdometryNode* odom_node, Path path, bool reversed, double wheelDiameter,
//         double fixedLookahead, double pathCompletionTolerance, bool gradualStop) :
//         m_odom_node(odom_node), m_controller(fixedLookahead, 20, 20, 0.05, path, reversed, pathCompletionTolerance, gradualStop, wheelDiameter) {
//     m_tank_drive = tank_drive;
// }

// void FollowPathAction::ActionInit() {

// }

// AutonAction::actionStatus FollowPathAction::Action() {
//     Pose pose = m_odom_node->getCurrentPose();
//     auto command = m_controller.Update(pose, pros::millis() / 1000.0);  

//     m_printString = "Current Pose X: " + to_string(pose.position(0)) + " Y: " + to_string(pose.position(1)) + " Rot: " + to_string(pose.angle.angle()) + 
//         " Remaining: " + to_string(m_controller.getRemainingLength()) + "\n";

//     m_tank_drive->Node::m_handle->logwarn(m_printString.c_str());

//     m_tank_drive->setDriveVelocity(command.left, command.right); 

//     if(m_controller.isDone()) {
//         return END;
//     } else {
//         return CONTINUE;
//     }
// }

// void FollowPathAction::ActionEnd() {
//     m_tank_drive->setDriveVelocity(0, 0);
// }