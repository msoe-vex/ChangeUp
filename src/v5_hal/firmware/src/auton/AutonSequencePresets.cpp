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

AutonNode* addActionsToPath_Goal4ToGoal1(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node) {
    AutonNode* delayedOuttakeGoal4 = new AutonNode(1.4, new WaitAction(1.4));
    initial_node->AddNext(delayedOuttakeGoal4);
    
    AutonNode* outtakeGoal4 = new AutonNode(0.5, new IntakeAction(intake_node, -MAX_MOTOR_VOLTAGE, 0.5));
    outtakeGoal4->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::REVERSE));
    delayedOuttakeGoal4->AddNext(outtakeGoal4);

    AutonNode* openIntakeAfterGoal4 = new AutonNode(0.4, new WaitAction(0.4));
    openIntakeAfterGoal4->AddAction(new OpenIntakesAction(intake_node));
    outtakeGoal4->AddNext(openIntakeAfterGoal4);

    AutonNode* delayedOpenIntakeAfterGoal4 = new AutonNode(0.1, new WaitAction(0.1));
    delayedOpenIntakeAfterGoal4->AddAction(new IntakeAction(intake_node));
    delayedOpenIntakeAfterGoal4->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING));
    openIntakeAfterGoal4->AddNext(delayedOpenIntakeAfterGoal4);

    AutonNode* closeIntakesOnBallAfterGoal4 = new AutonNode(2.0, new WaitAction(2.0));
    closeIntakesOnBallAfterGoal4->AddAction(new OpenIntakesAction(intake_node, false));
    delayedOpenIntakeAfterGoal4->AddNext(closeIntakesOnBallAfterGoal4);

    AutonNode* stopBottomRollerBeforeGoal1 = new AutonNode(0.1, new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING_TOP));
    closeIntakesOnBallAfterGoal4->AddNext(stopBottomRollerBeforeGoal1);

    return stopBottomRollerBeforeGoal1;
}

AutonNode* addActionsToPath_Goal1ToGoal2(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node) {
    return nullptr;
}

AutonNode* addActionsToPath_Goal2ToGoal3(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node) {
    return nullptr;
}

AutonNode* addActionsToPath_Goal3ToGoal6(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node) {
    return nullptr;
}