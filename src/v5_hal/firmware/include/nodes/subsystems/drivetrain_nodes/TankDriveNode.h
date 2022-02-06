#pragma once

#include "nodes/NodeManager.h"
#include "api.h"
#include "nodes/actuator_nodes/MotorNode.h"
#include "nodes/sensor_nodes/ControllerNode.h"
#include "nodes/subsystems/drivetrain_nodes/IDriveNode.h"
#include "nodes/sensor_nodes/InertialSensorNode.h"

class TankDriveNode : public IDriveNode {
public: 
    struct TankEightMotors {
        MotorNode* left_motor_1;
        MotorNode* left_motor_2;
        MotorNode* left_motor_3;
        MotorNode* left_motor_4;
        MotorNode* right_motor_1;
        MotorNode* right_motor_2;
        MotorNode* right_motor_3;
        MotorNode* right_motor_4;
    };

    TankDriveNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, 
        InertialSensorNode* inertial_sensor, TankEightMotors motors);

    void initialize();

    void resetEncoders();

    IDriveNode::FourMotorDriveEncoderVals getIntegratedEncoderVals();

    void setDriveVoltage(int left_voltage, int right_voltage);

    void setDriveVelocity(float left_velocity, float right_velocity);

    void setDriveDistancePID(double left_distance, double right_distance, int max_velocity);

    void teleopPeriodic();

    void autonPeriodic();

    int getRightDistancePID();

    int getLeftDistancePID();

    ~TankDriveNode();

private:
    pros::Controller* m_controller;

    InertialSensorNode* m_inertial_sensor;
    
    TankEightMotors m_motors;

    std::string m_handle_name;

    void m_setLeftVoltage(int voltage);

    void m_setRightVoltage(int voltage);

    void m_setLeftVelocity(float velocity);

    void m_setRightVelocity(float velocity);

    void m_setLeftDistancePID(double distance, int max_velocity);

    void m_setRightDistancePID(double distance, int max_velocity);
};
