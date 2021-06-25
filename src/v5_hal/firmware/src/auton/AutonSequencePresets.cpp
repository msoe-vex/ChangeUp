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

    // AutonNode* intakeOff = new AutonNode(0.1, new IntakeAction(intake_node, 0));
    // initial_node->AddNext(intakeOff);


    AutonNode* holdSecondBall = new AutonNode(0.5, new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING, 0.5));
    scoreFirstBall->AddNext(holdSecondBall);


    AutonNode* splitConveyor = new AutonNode(0.5, new UpdateConveyorStateAction(conveyor_node, ConveyorNode::SPLIT, 0.5));
    holdSecondBall->AddNext(splitConveyor);

    // AutonNode* waitForIntake = new AutonNode(0.7, new WaitAction(0.7));
    // intakeOff->AddNext(waitForIntake);


    AutonNode* holdBlueBalls = new AutonNode(0.5, new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING_TOP, 0.5));
    splitConveyor->AddNext(holdBlueBalls);

    // AutonNode* finalIntakeOn = new AutonNode(2.0, new IntakeAction(intake_node, MAX_MOTOR_VOLTAGE, 2.0));
    // waitForIntake->AddNext(finalIntakeOn);

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
    AutonNode* delayedOuttakeGoal1 = new AutonNode(0.5, new WaitAction(0.5));
    initial_node->AddNext(delayedOuttakeGoal1);
    
    AutonNode* outtakeGoal1 = new AutonNode(0.7, new IntakeAction(intake_node, -MAX_MOTOR_VOLTAGE, 0.7));
    outtakeGoal1->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::REVERSE));
    delayedOuttakeGoal1->AddNext(outtakeGoal1);

    AutonNode* openIntakeAfterGoal1 = new AutonNode(0.1, new WaitAction(0.1));
    openIntakeAfterGoal1->AddAction(new OpenIntakesAction(intake_node));
    outtakeGoal1->AddNext(openIntakeAfterGoal1);

    AutonNode* delayedOpenIntakeAfterGoal1 = new AutonNode(0.7, new WaitAction(0.7));
    delayedOpenIntakeAfterGoal1->AddAction(new IntakeAction(intake_node));
    delayedOpenIntakeAfterGoal1->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING));
    openIntakeAfterGoal1->AddNext(delayedOpenIntakeAfterGoal1);

    AutonNode* closeIntakesOnBall1AfterGoal1 = new AutonNode(0.4, new WaitAction(0.4));
    closeIntakesOnBall1AfterGoal1->AddAction(new OpenIntakesAction(intake_node, false));
    delayedOpenIntakeAfterGoal1->AddNext(closeIntakesOnBall1AfterGoal1);

    AutonNode* openIntakesAfterBall1Goal1 = new AutonNode(0.7, new WaitAction(0.7));
    openIntakesAfterBall1Goal1->AddAction(new OpenIntakesAction(intake_node));
    closeIntakesOnBall1AfterGoal1->AddNext(openIntakesAfterBall1Goal1);

    AutonNode* closeIntakesOnBall2AfterGoal1 = new AutonNode(0.2, new WaitAction(0.2));
    closeIntakesOnBall2AfterGoal1->AddAction(new OpenIntakesAction(intake_node, false));
    openIntakesAfterBall1Goal1->AddNext(closeIntakesOnBall2AfterGoal1);

    AutonNode* switchToHoldingTopConveyorMode = new AutonNode(0.1, new WaitAction(0.1));
    switchToHoldingTopConveyorMode->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING_TOP));
    closeIntakesOnBall2AfterGoal1->AddNext(switchToHoldingTopConveyorMode);

    return switchToHoldingTopConveyorMode;
}

AutonNode* addActionsToPath_Goal2ToGoal3(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node) {
    AutonNode* delayedOuttakeGoal2 = new AutonNode(0.5, new WaitAction(0.5));
    initial_node->AddNext(delayedOuttakeGoal2);
    
    AutonNode* outtakeGoal2 = new AutonNode(0.7, new IntakeAction(intake_node, -MAX_MOTOR_VOLTAGE, 0.7));
    outtakeGoal2->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::REVERSE));
    delayedOuttakeGoal2->AddNext(outtakeGoal2);

    AutonNode* openIntakeAfterGoal2 = new AutonNode(0.3, new WaitAction(0.3));
    openIntakeAfterGoal2->AddAction(new OpenIntakesAction(intake_node));
    outtakeGoal2->AddNext(openIntakeAfterGoal2);

    AutonNode* delayedOpenIntakeAfterGoal2 = new AutonNode(0.8, new WaitAction(0.8));
    delayedOpenIntakeAfterGoal2->AddAction(new IntakeAction(intake_node));
    delayedOpenIntakeAfterGoal2->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING));
    openIntakeAfterGoal2->AddNext(delayedOpenIntakeAfterGoal2);

    AutonNode* closeIntakesOnBall1AfterGoal2 = new AutonNode(0.7, new WaitAction(0.7));
    closeIntakesOnBall1AfterGoal2->AddAction(new OpenIntakesAction(intake_node, false));
    delayedOpenIntakeAfterGoal2->AddNext(closeIntakesOnBall1AfterGoal2);

    AutonNode* openIntakesAfterBall1Goal2 = new AutonNode(0.8, new WaitAction(0.8));
    openIntakesAfterBall1Goal2->AddAction(new OpenIntakesAction(intake_node));
    closeIntakesOnBall1AfterGoal2->AddNext(openIntakesAfterBall1Goal2);

    AutonNode* closeIntakesOnBall2AfterGoal2 = new AutonNode(0.4, new WaitAction(0.4));
    closeIntakesOnBall2AfterGoal2->AddAction(new OpenIntakesAction(intake_node, false));
    openIntakesAfterBall1Goal2->AddNext(closeIntakesOnBall2AfterGoal2);

    AutonNode* switchToHoldingTopConveyorMode = new AutonNode(0.1, new WaitAction(0.1));
    switchToHoldingTopConveyorMode->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING_TOP));
    closeIntakesOnBall2AfterGoal2->AddNext(switchToHoldingTopConveyorMode);    
    
    return switchToHoldingTopConveyorMode;
}

AutonNode* addActionsToPath_Goal3ToGoal6(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node) {
    AutonNode* delayedOuttakeGoal3 = new AutonNode(0.5, new WaitAction(0.5));
    initial_node->AddNext(delayedOuttakeGoal3);
    
    AutonNode* outtakeGoal3 = new AutonNode(0.7, new IntakeAction(intake_node, -MAX_MOTOR_VOLTAGE, 0.7));
    outtakeGoal3->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::REVERSE));
    delayedOuttakeGoal3->AddNext(outtakeGoal3);

    AutonNode* openIntakeAfterGoal3 = new AutonNode(0.3, new WaitAction(0.3));
    openIntakeAfterGoal3->AddAction(new OpenIntakesAction(intake_node));
    outtakeGoal3->AddNext(openIntakeAfterGoal3);

    AutonNode* delayedOpenIntakeAfterGoal3 = new AutonNode(1.0, new WaitAction(1.0));
    delayedOpenIntakeAfterGoal3->AddAction(new IntakeAction(intake_node));
    delayedOpenIntakeAfterGoal3->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING));
    openIntakeAfterGoal3->AddNext(delayedOpenIntakeAfterGoal3);

    AutonNode* closeIntakesOnBallAfterGoal3 = new AutonNode(0.1, new WaitAction(0.1));
    closeIntakesOnBallAfterGoal3->AddAction(new OpenIntakesAction(intake_node, false));
    delayedOpenIntakeAfterGoal3->AddNext(closeIntakesOnBallAfterGoal3);

    return closeIntakesOnBallAfterGoal3;
}

AutonNode* addActionsToPath_Goal6ToGoal9(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node) {
    AutonNode* delayedOuttakeGoal6 = new AutonNode(1.4, new WaitAction(1.4));
    initial_node->AddNext(delayedOuttakeGoal6);
    
    AutonNode* outtakeGoal6 = new AutonNode(0.5, new IntakeAction(intake_node, -MAX_MOTOR_VOLTAGE, 0.5));
    outtakeGoal6->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::REVERSE));
    delayedOuttakeGoal6->AddNext(outtakeGoal6);

    AutonNode* openIntakeAfterGoal6 = new AutonNode(0.4, new WaitAction(0.4));
    openIntakeAfterGoal6->AddAction(new OpenIntakesAction(intake_node));
    outtakeGoal6->AddNext(openIntakeAfterGoal6);

    AutonNode* delayedOpenIntakeAfterGoal6 = new AutonNode(0.1, new WaitAction(0.1));
    delayedOpenIntakeAfterGoal6->AddAction(new IntakeAction(intake_node));
    delayedOpenIntakeAfterGoal6->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING));
    openIntakeAfterGoal6->AddNext(delayedOpenIntakeAfterGoal6);

    AutonNode* closeIntakesOnBallAfterGoal6 = new AutonNode(2.0, new WaitAction(2.0));
    closeIntakesOnBallAfterGoal6->AddAction(new OpenIntakesAction(intake_node, false));
    delayedOpenIntakeAfterGoal6->AddNext(closeIntakesOnBallAfterGoal6);

    AutonNode* stopBottomRollerBeforeGoal9 = new AutonNode(0.1, new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING_TOP));
    closeIntakesOnBallAfterGoal6->AddNext(stopBottomRollerBeforeGoal9);

    return stopBottomRollerBeforeGoal9;
}

AutonNode* addActionsToPath_Goal9ToGoal8(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node) {
    AutonNode* delayedOuttakeGoal9 = new AutonNode(0.3, new WaitAction(0.3));
    initial_node->AddNext(delayedOuttakeGoal9);
    
    AutonNode* outtakeGoal9 = new AutonNode(0.7, new IntakeAction(intake_node, -MAX_MOTOR_VOLTAGE, 0.7));
    outtakeGoal9->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::REVERSE));
    delayedOuttakeGoal9->AddNext(outtakeGoal9);

    AutonNode* openIntakeAfterGoal9 = new AutonNode(0.1, new WaitAction(0.1));
    openIntakeAfterGoal9->AddAction(new OpenIntakesAction(intake_node));
    outtakeGoal9->AddNext(openIntakeAfterGoal9);

    AutonNode* delayedOpenIntakeAfterGoal9 = new AutonNode(0.7, new WaitAction(0.7));
    delayedOpenIntakeAfterGoal9->AddAction(new IntakeAction(intake_node));
    delayedOpenIntakeAfterGoal9->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING));
    openIntakeAfterGoal9->AddNext(delayedOpenIntakeAfterGoal9);

    AutonNode* closeIntakesOnBall1AfterGoal9 = new AutonNode(0.4, new WaitAction(0.4));
    closeIntakesOnBall1AfterGoal9->AddAction(new OpenIntakesAction(intake_node, false));
    delayedOpenIntakeAfterGoal9->AddNext(closeIntakesOnBall1AfterGoal9);

    AutonNode* openIntakesAfterBall1Goal9 = new AutonNode(0.7, new WaitAction(0.7));
    openIntakesAfterBall1Goal9->AddAction(new OpenIntakesAction(intake_node));
    closeIntakesOnBall1AfterGoal9->AddNext(openIntakesAfterBall1Goal9);

    AutonNode* closeIntakesOnBall2AfterGoal9 = new AutonNode(0.2, new WaitAction(0.2));
    closeIntakesOnBall2AfterGoal9->AddAction(new OpenIntakesAction(intake_node, false));
    openIntakesAfterBall1Goal9->AddNext(closeIntakesOnBall2AfterGoal9);

    AutonNode* switchToHoldingTopConveyorMode = new AutonNode(0.1, new WaitAction(0.1));
    switchToHoldingTopConveyorMode->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING_TOP));
    closeIntakesOnBall2AfterGoal9->AddNext(switchToHoldingTopConveyorMode);

    return switchToHoldingTopConveyorMode;
}

AutonNode* addActionsToPath_Goal8ToGoal7(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node) {
    AutonNode* delayedOuttakeGoal8 = new AutonNode(0.5, new WaitAction(0.5));
    initial_node->AddNext(delayedOuttakeGoal8);
    
    AutonNode* outtakeGoal8 = new AutonNode(0.7, new IntakeAction(intake_node, -MAX_MOTOR_VOLTAGE, 0.7));
    outtakeGoal8->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::REVERSE));
    delayedOuttakeGoal8->AddNext(outtakeGoal8);

    AutonNode* openIntakeAfterGoal8 = new AutonNode(0.3, new WaitAction(0.3));
    openIntakeAfterGoal8->AddAction(new OpenIntakesAction(intake_node));
    outtakeGoal8->AddNext(openIntakeAfterGoal8);

    AutonNode* delayedOpenIntakeAfterGoal8 = new AutonNode(0.8, new WaitAction(0.8));
    delayedOpenIntakeAfterGoal8->AddAction(new IntakeAction(intake_node));
    delayedOpenIntakeAfterGoal8->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING));
    openIntakeAfterGoal8->AddNext(delayedOpenIntakeAfterGoal8);

    AutonNode* closeIntakesOnBall1AfterGoal8 = new AutonNode(0.7, new WaitAction(0.7));
    closeIntakesOnBall1AfterGoal8->AddAction(new OpenIntakesAction(intake_node, false));
    delayedOpenIntakeAfterGoal8->AddNext(closeIntakesOnBall1AfterGoal8);

    AutonNode* openIntakesAfterBall1Goal8 = new AutonNode(0.8, new WaitAction(0.8));
    openIntakesAfterBall1Goal8->AddAction(new OpenIntakesAction(intake_node));
    closeIntakesOnBall1AfterGoal8->AddNext(openIntakesAfterBall1Goal8);

    AutonNode* closeIntakesOnBall2AfterGoal8 = new AutonNode(0.4, new WaitAction(0.4));
    closeIntakesOnBall2AfterGoal8->AddAction(new OpenIntakesAction(intake_node, false));
    openIntakesAfterBall1Goal8->AddNext(closeIntakesOnBall2AfterGoal8);

    AutonNode* switchToHoldingTopConveyorMode = new AutonNode(0.1, new WaitAction(0.1));
    switchToHoldingTopConveyorMode->AddAction(new UpdateConveyorStateAction(conveyor_node, ConveyorNode::HOLDING_TOP));
    closeIntakesOnBall2AfterGoal8->AddNext(switchToHoldingTopConveyorMode);    
    
    return switchToHoldingTopConveyorMode;
}

AutonNode* addActionsToPath_Goal7ToGoal5(AutonNode* initial_node, IntakeNode* intake_node, ConveyorNode* conveyor_node) {
    return nullptr;
}