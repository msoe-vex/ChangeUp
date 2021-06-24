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

/**
 * This function is used to generate a sequence to score balls in a goal.
 * This should follow the node that makes the robot drive into the goal
 */
AutonNode* getSingleGoalScoringSequence(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);

AutonNode* getDoubleGoalScoringSequence(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);

AutonNode* addActionsToPath_Goal4ToGoal1(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);

AutonNode* addActionsToPath_Goal1ToGoal2(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);

AutonNode* addActionsToPath_Goal2ToGoal3(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node);
