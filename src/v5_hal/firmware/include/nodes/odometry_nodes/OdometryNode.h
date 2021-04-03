#pragma once

#include "nodes/NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "nodes/subsystems/drivetrain_nodes/TankDriveNode.h"
#include "nodes/sensor_nodes/ADIEncoderNode.h"
#include "nodes/sensor_nodes/InertialSensorNode.h"
#include "nodes/actuator_nodes/MotorNode.h"
#include "odometry/Odometry.h"
#include "odometry/FollowerOdometry.h"
#include "odometry/TankOdometry.h"
#include "util/Encoders.h"
#include "eigen/Eigen/Dense"
#include "util/Logger.h"

class OdometryNode : public Node { 
public:
    enum OdomConfig {
        FOLLOWER, TANK
    };

    OdometryNode(NodeManager* node_manager, std::string handle_name, ADIEncoderNode* odom_encoder_1, 
        ADIEncoderNode* odom_encoder_2, InertialSensorNode* inertial_sensor, OdomConfig odom_config);

    OdometryNode(NodeManager* node_manager, std::string handle_name, MotorNode* motor_1, MotorNode* motor_2, 
        InertialSensorNode* inertial_sensor, OdomConfig odom_config);

    void initialize();

    void setCurrentPose(Pose pose);

    Pose getCurrentPose();

    void teleopPeriodic();

    void autonPeriodic();

    ~OdometryNode();

private:    
    std::string m_handle_name;

    OdomConfig m_odom_config;

    ADIEncoderNode* m_odom_encoder_1;
    ADIEncoderNode* m_odom_encoder_2;
    MotorNode* m_motor_1;
    MotorNode* m_motor_2;
    InertialSensorNode* m_inertial_sensor_node;
    Eigen::Rotation2Dd m_current_angle_offset;

    Odometry* m_odom;

    Odometry* m_getOdomClass(OdomConfig odom_config);
};
