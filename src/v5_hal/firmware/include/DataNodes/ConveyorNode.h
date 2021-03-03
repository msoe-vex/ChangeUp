#pragma once

#include "NodeManager.h"
#include "api.h"
#include "DataNodes/MotorNode.h"
#include "DataNodes/ADIAnalogInNode.h"
#include "DataNodes/ControllerNode.h"
#include "DataNodes/ADIDigitalOutNode.h"

class ConveyorNode : public Node {
public:
    enum ConveyorState {
        STOPPED, HOLDING, SCORING, REVERSE
    };

    ConveyorNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, MotorNode* left_intake,
        MotorNode* right_intake, MotorNode* bottom_conveyor_motor, MotorNode* ejection_roller_motor, MotorNode* top_conveyor_motor,
        ADIAnalogInNode* bottom_conveyor_sensor, ADIAnalogInNode* middle_conveyor_sensor, ADIAnalogInNode* top_conveyor_sensor,
        ADIDigitalOutNode* digital_out_node);

    void setIntakeVoltage(int voltage);

    void setBottomConveyorVoltage(int voltage);

    void setEjectionRollerVoltage(int voltage);

    void setTopConveyorVoltage(int voltage);

    void setConveyorState(ConveyorState conveyorState);

    int getNumBallsStored();

    void openIntakes(int open);

    void initialize();

    void teleopPeriodic();

    void autonPeriodic();

    ~ConveyorNode();

private:
    ConveyorState m_current_conveyor_state = STOPPED;

    const int BALL_PRESENT_THRESHOLD = 2700;

    pros::Controller* m_controller;
    MotorNode* m_left_intake;
    MotorNode* m_right_intake;
    MotorNode* m_bottom_conveyor_motor;
    MotorNode* m_ejection_roller_motor;
    MotorNode* m_top_conveyor_motor;
    ADIAnalogInNode* m_bottom_conveyor_sensor;
    ADIAnalogInNode* m_middle_conveyor_sensor;
    ADIAnalogInNode* m_top_conveyor_sensor;
    ADIDigitalOutNode* m_intake_pneumatics;

    std::string m_handle_name;

    bool m_enableStateMachine = false;

    bool m_open = false;

    void m_updateConveyorHoldingState();
};
