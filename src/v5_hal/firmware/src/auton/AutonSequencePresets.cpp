#include "auton/AutonSequencePresets.h"

AutonNode* getSingleGoalScoringSequence(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node) {
    AutonNode* scoreFirstBall = new AutonNode(0.4, new UpdateConveyorStateAction(conveyor_node, ConveyorNode::SCORING_TOP, 0.4));
    initial_node->AddNext(scoreFirstBall);

    AutonNode* intakeOff = new AutonNode(0.1, new IntakeAction(intake_node, 0));
    initial_node->AddNext(intakeOff);

    AutonNode* holdSecondBall = new AutonNode(0.5, new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING_TOP, 0.5));
    scoreFirstBall->AddNext(holdSecondBall);

    AutonNode* waitForIntake = new AutonNode(0.25, new WaitAction(0.25));
    intakeOff->AddNext(waitForIntake);

    AutonNode* finalIntakeOn = new AutonNode(0.5, new IntakeAction(intake_node, MAX_MOTOR_VOLTAGE, 2.0));
    waitForIntake->AddNext(finalIntakeOn);

    return finalIntakeOn;
}

AutonNode* getDoubleGoalScoringSequence(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node) {
    AutonNode* scoreFirstBall = new AutonNode(0.2, new UpdateConveyorStateAction(conveyor_node, ConveyorNode::SCORING, 0.2));
    initial_node->AddNext(scoreFirstBall);

    AutonNode* intakeOff = new AutonNode(0.1, new IntakeAction(intake_node, 0));
    initial_node->AddNext(intakeOff);


    AutonNode* holdSecondBall = new AutonNode(0.5, new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING, 0.5));
    scoreFirstBall->AddNext(holdSecondBall);


    AutonNode* splitConveyor = new AutonNode(0.5, new UpdateConveyorStateAction(conveyor_node, ConveyorNode::SPLIT, 0.5));
    holdSecondBall->AddNext(splitConveyor);

    AutonNode* waitForIntake = new AutonNode(0.7, new WaitAction(0.7));
    intakeOff->AddNext(waitForIntake);


    AutonNode* holdBlueBalls = new AutonNode(1.5, new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING_TOP, 1.5));
    splitConveyor->AddNext(holdBlueBalls);

    AutonNode* finalIntakeOn = new AutonNode(2.0, new IntakeAction(intake_node, MAX_MOTOR_VOLTAGE, 2.0));
    waitForIntake->AddNext(finalIntakeOn);

    return holdBlueBalls;
}