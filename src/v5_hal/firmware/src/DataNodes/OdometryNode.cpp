#include "DataNodes/OdometryNode.h"

OdometryNode::OdometryNode(NodeManager* node_manager, std::string handle_name, TankDriveNode* chassis, 
        ADIEncoderNode* odom_encoder_1, ADIEncoderNode* odom_encoder_2, InertialSensorNode* inertial_sensor_node, 
        OdomConfig odom_config) : Node(node_manager, 10), m_handle_name(handle_name), m_chassis(chassis),
        m_odom_encoder_1(odom_encoder_1), m_odom_encoder_2(odom_encoder_2), m_inertial_sensor_node(inertial_sensor_node),
        m_odom_config(odom_config) {
    m_odom = m_getOdomClass(odom_config);
}

Odometry* OdometryNode::m_getOdomClass(OdomConfig config) {
    Odometry::EncoderConfig adi_encoder_config = {0., 360., 2.8};

    switch (config) {
        case FOLLOWER:
            return new FollowerOdometry(adi_encoder_config, adi_encoder_config);
        case TANK:
            return new TankOdometry(adi_encoder_config, adi_encoder_config);
        default:
            Node::m_handle->logerror("Error creating odometry instance in OdometryNode.cpp");
            return new FollowerOdometry(adi_encoder_config, adi_encoder_config);
    }
}

void OdometryNode::initialize() {

}

Pose OdometryNode::getCurrentPose() {
    return m_odom->GetPose();
}

void OdometryNode::teleopPeriodic() {
    Rotation2Dd current_angle(m_inertial_sensor_node->getYaw());

    m_odom->Update(m_odom_encoder_1->getValue(), m_odom_encoder_2->getValue(), current_angle);
}

void OdometryNode::autonPeriodic() {
    Rotation2Dd current_angle(m_inertial_sensor_node->getYaw());

    m_odom->Update(m_odom_encoder_1->getValue(), m_odom_encoder_2->getValue(), current_angle);
}

OdometryNode::~OdometryNode() {
    delete m_odom;
}