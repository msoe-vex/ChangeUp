#pragma once

#include "api.h"
#include "nodes/subsystems/IntakeNode.h"
#include "nodes/subsystems/ConveyorNode.h"
#include "nodes/subsystems/drivetrain_nodes/IDriveNode.h"
#include "auton/Auton.h"
#include "auton/AutonSequence.h"
#include "auton/auton_actions/OpenIntakesAction.h"
#include "auton/auton_actions/IntakeAction.h"
#include "auton/auton_actions/UpdateConveyorStateAction.h"
#include "auton/auton_actions/TurnToAngleAction.h"
#include "auton/auton_actions/DriveToPoseAction.h"
#include "auton/auton_actions/LiftGoalPlateAction.h"

/**
 * This function is used to generate a sequence to score balls in a goal.
 * This should follow the node that makes the robot drive into the goal
 */
AutonNode* getSingleGoalScoringSequence(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);

AutonNode* getDoubleGoalScoringSequence(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);

AutonNode* getMiddleGoalScoringSequence(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);

AutonNode* addActionsToPath_Goal4ToGoal1(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);

AutonNode* addActionsToPath_Goal1ToGoal2(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);

AutonNode* addActionsToPath_Goal2ToGoal3(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);

AutonNode* addActionsToPath_Goal3ToGoal6(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);

AutonNode* addActionsToPath_Goal6ToGoal9(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);

AutonNode* addActionsToPath_Goal9ToGoal8(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);

AutonNode* addActionsToPath_Goal8ToGoal7(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);

AutonNode* addActionsToPath_Goal7ToGoal5(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);