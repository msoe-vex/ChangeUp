#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "DataNodes/TankDriveNode.h"
#include "DataNodes/ADIEncoderNode.h"
#include "DataNodes/InertialSensorNode.h"
#include "odometry/Odometry.h"
#include "odometry/FollowerOdometry.h"
#include "odometry/TankOdometry.h"

class OdometryNode : public Node { 
public:
    enum OdomConfig {
        FOLLOWER, TANK
    };

    OdometryNode(NodeManager* node_manager, std::string handle_name, TankDriveNode* chassis, 
        ADIEncoderNode* odom_encoder_1, ADIEncoderNode* odom_encoder_2, InertialSensorNode* inertial_sensor,
        OdomConfig odom_config);

    void initialize();

    Pose getCurrentPose();

    void teleopPeriodic();

    void autonPeriodic();

    ~OdometryNode();

private:    
    std::string m_handle_name;

    OdomConfig m_odom_config;

    TankDriveNode* m_chassis;
    ADIEncoderNode* m_odom_encoder_1;
    ADIEncoderNode* m_odom_encoder_2;
    InertialSensorNode* m_inertial_sensor;

    Odometry* m_odom;

    Odometry* m_getOdomClass(OdomConfig odom_config);
};
