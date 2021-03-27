#include "auton/AutonSequencePresets.h"

AutonNode* getGoalScoringSequence(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node) {
    AutonSequence* goal_scoring_sequence = new AutonSequence(initial_node);

    goal_scoring_sequence->AddAction(new OpenIntakesAction(intake_node));
    goal_scoring_sequence->AddAction(new IntakeAction(intake_node, 0));

    goal_scoring_sequence->AddNext(new AutonNode(3., new WaitAction(3.)));
    goal_scoring_sequence->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::SCORING));

    goal_scoring_sequence->AddNext(new AutonNode(3., new IntakeAction(intake_node, 12000, 3.)));
    goal_scoring_sequence->AddAction(new OpenIntakesAction(intake_node, false));
    goal_scoring_sequence->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING));

    goal_scoring_sequence->AddNext(new AutonNode(0.5, new OpenIntakesAction(intake_node)));

    return goal_scoring_sequence->GetSequence().tailNode;
}