#pragma once

#include "nodes/NodeManager.h"
#include "api.h"
#include "nodes/actuator_nodes/MotorNode.h"
#include "nodes/sensor_nodes/ControllerNode.h"
#include "nodes/actuator_nodes/ADIDigitalOutNode.h"
#include "util/Constants.h"

class IntakeNode : public Node {
private:
    pros::Controller* m_controller;
    MotorNode* m_left_intake;
    MotorNode* m_right_intake;
    ADIDigitalOutNode* m_intake_deploy;
    ADIDigitalOutNode* m_intake_open;

    std::string m_handle_name;

    bool m_open = false;

public:
    IntakeNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, MotorNode* left_intake,
        MotorNode* right_intake, ADIDigitalOutNode* intake_deploy, ADIDigitalOutNode* intake_open);

    void setIntakeVoltage(int voltage);

    void openIntakes(int open);

    void deployIntakes(int open);

    void initialize();

    void teleopPeriodic();

    void autonPeriodic();

    ~IntakeNode();
};
