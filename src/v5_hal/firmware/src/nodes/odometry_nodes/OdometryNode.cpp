#include "nodes/odometry_nodes/OdometryNode.h"

OdometryNode::OdometryNode(NodeManager* node_manager, std::string handle_name, 
        ADIEncoderNode* odom_encoder_1, ADIEncoderNode* odom_encoder_2, InertialSensorNode* inertial_sensor_node, 
        OdomConfig odom_config) : Node(node_manager, 50), m_handle_name(handle_name),
        m_odom_encoder_1(odom_encoder_1), m_odom_encoder_2(odom_encoder_2), m_inertial_sensor_node(inertial_sensor_node),
        m_current_angle_offset(0), m_odom_config(odom_config) {
    m_odom = m_getOdomClass(odom_config);
}

OdometryNode::OdometryNode(NodeManager* node_manager, std::string handle_name, 
        MotorNode* motor_1, MotorNode* motor_2, InertialSensorNode* inertial_sensor_node, 
        OdomConfig odom_config) : Node(node_manager, 50), m_handle_name(handle_name),
        m_motor_1(motor_1), m_motor_2(motor_2), m_inertial_sensor_node(inertial_sensor_node),
        m_current_angle_offset(0), m_odom_config(odom_config) {
    m_odom = m_getOdomClass(odom_config);
}

Odometry* OdometryNode::m_getOdomClass(OdomConfig config) {
    EncoderConfig adi_encoder_config = {0., 360., 2.8};
    EncoderConfig v5_integrated_encoder_config = {0., 900., 4.0625};

    switch (config) {
        case FOLLOWER:
            return new FollowerOdometry(adi_encoder_config, adi_encoder_config);
        case TANK:
            return new TankOdometry(v5_integrated_encoder_config, v5_integrated_encoder_config);
        default:
            Node::m_handle->logerror("Error creating odometry instance in OdometryNode.cpp");
            return new FollowerOdometry(adi_encoder_config, adi_encoder_config);
    }
}

void OdometryNode::initialize() {
    // Calibrate the gyro, and reset the orientation with the correct gyro angle
    m_inertial_sensor_node->reset();
	setCurrentPose(Pose(Vector2d(0, 0), m_inertial_sensor_node->getYaw()));
}

void OdometryNode::setCurrentPose(Pose pose) {
    m_odom->SetCurrentPose(pose);
}

Pose OdometryNode::getCurrentPose() {
    return m_odom->GetPose();
}

void OdometryNode::teleopPeriodic() {
    m_odom->Update(m_odom_encoder_1->getValue(), m_odom_encoder_2->getValue(), m_inertial_sensor_node->getYaw());
}

void OdometryNode::autonPeriodic() {
    m_odom->Update(m_odom_encoder_1->getValue(), m_odom_encoder_2->getValue(), m_inertial_sensor_node->getYaw());
}

OdometryNode::~OdometryNode() {
    delete m_odom;
}