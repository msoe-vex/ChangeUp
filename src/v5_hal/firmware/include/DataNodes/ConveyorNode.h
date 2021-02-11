#pragma once

#include "NodeManager.h"
#include "api.h"
#include "DataNodes/MotorNode.h"
#include "DataNodes/ADIAnalogInNode.h"
#include "DataNodes/ControllerNode.h"

class ConveyorNode : public Node {
private:
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

    std::string m_handle_name;

    bool m_enableStateMachine = false;

    void m_setIntakeVoltage(int voltage);

    void m_setBottomConveyorVoltage(int voltage);

    void m_setEjectionRollerVoltage(int voltage);

    void m_setTopConveyorVoltage(int voltage);

    void m_updateConveyorStateMachine();

    void deploy();

public: 
    ConveyorNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, MotorNode* left_intake, 
        MotorNode* right_intake, MotorNode* bottom_conveyor_motor, MotorNode* ejection_roller_motor, MotorNode* top_conveyor_motor, 
        ADIAnalogInNode* bottom_conveyor_sensor, ADIAnalogInNode* middle_conveyor_sensor, ADIAnalogInNode* top_conveyor_sensor);

    void initialize();

    void periodic();

    ~ConveyorNode();
};
