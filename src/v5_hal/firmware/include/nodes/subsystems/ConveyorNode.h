#pragma once

#include "nodes/NodeManager.h"
#include "api.h"
#include "nodes/actuator_nodes/MotorNode.h"
#include "nodes/sensor_nodes/ADIAnalogInNode.h"
#include "nodes/sensor_nodes/ControllerNode.h"
#include "nodes/actuator_nodes/ADIDigitalOutNode.h"
#include "util/Constants.h"

class ConveyorNode : public Node {
public:
    enum ConveyorState {
        STOPPED, HOLDING, SCORING, REVERSE
    };

    ConveyorNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, MotorNode* bottom_conveyor_motor, 
        MotorNode* top_conveyor_motor, ADIAnalogInNode* bottom_conveyor_sensor, ADIAnalogInNode* top_conveyor_sensor);

    void setBottomConveyorVoltage(int voltage);

    void setTopConveyorVoltage(int voltage);

    void setConveyorState(ConveyorState conveyorState);

    int getNumBallsStored();

    void initialize();

    void teleopPeriodic();

    void autonPeriodic();

    ~ConveyorNode();

private:
    ConveyorState m_current_conveyor_state = STOPPED;

    pros::Controller* m_controller;
    MotorNode* m_bottom_conveyor_motor;
    MotorNode* m_top_conveyor_motor;
    ADIAnalogInNode* m_bottom_conveyor_sensor;
    ADIAnalogInNode* m_top_conveyor_sensor;

    std::string m_handle_name;

    bool m_enableStateMachine = false;

    void m_updateConveyorHoldingState();
};
