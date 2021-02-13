#include "FollowPathAction.h"


FollowPathAction::FollowPathAction(TankDriveNode* tank_drive, Path path, double wheelDiameter, bool reversed,
                             double fixedLookahead, double pathCompletionTolerance, bool gradualStop) :
            m_controller(fixedLookahead, 0.1, 0.01, path, reversed, pathCompletionTolerance, gradualStop, wheelDiameter) {
    m_tank_drive = tank_drive;
}

void FollowPathAction::ActionInit() {

}

AutonAction::actionStatus FollowPathAction::Action() {
    auto command = m_controller.Update(TankOdometry::GetInstance()->GetPose(), pros::millis() / 1000.0);

    m_tank_drive->setDriveVelocity(command.left, command.right); 

    if(m_controller.isDone()) {
        return END;
    } else {
        return CONTINUE;
    }
}

void FollowPathAction::ActionEnd() {
    m_tank_drive->setDriveVelocity(0, 0);
}