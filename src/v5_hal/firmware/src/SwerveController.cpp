#include "SwerveController.h"


SwerveController::SwerveController(NodeManager * node_manager, Eigen::Vector2d left_module_location, Eigen::Vector2d right_module_location, Eigen::Vector2d rear_module_location,
    double rotation_angle_threshold, double max_velocity, double max_rotation_velocity, double kP, double kI, double kD) : 
    leftSwerveModule(left_module_location, rotation_angle_threshold, max_velocity, max_rotation_velocity, kP, kI, kD),
    rightSwerveModule(right_module_location, rotation_angle_threshold, max_velocity, max_rotation_velocity, kP, kI, kD),
    rearSwerveModule(rear_module_location, rotation_angle_threshold, max_velocity, max_rotation_velocity, kP, kI, kD),
    Node(node_manager, 50) {

    m_rightSetpointPub = new ros::Publisher("rightSwerveSetpoint", &m_rightSetpointMsg);
    m_leftSetpointPub = new ros::Publisher("leftSwerveSetpoint", &m_leftSetpointMsg);
    m_rearSetpointPub = new ros::Publisher("rearSwerveSetpoint", &m_rearSetpointMsg);

    m_rightActualPub = new ros::Publisher("rightSwerveActual", &m_rightActualMsg);
    m_leftActualPub = new ros::Publisher("leftSwerveActual", &m_leftActualMsg);
    m_rearActualPub = new ros::Publisher("rearSwerveActual", &m_rearActualMsg);

    PID_recieved = false;
}

void SwerveController::assignActualAngle(int left_pot, int right_pot, int rear_pot) {
    left_actual_angle = Eigen::Rotation2Dd((((((float)left_pot - LEFT_POT_OFFSET) / 4095.0) * M_PI) * 2.0) * -1);
    right_actual_angle = Eigen::Rotation2Dd((((((float)right_pot - RIGHT_POT_OFFSET) / 4095.0) * M_PI) * 2.0) * -1);
    rear_actual_angle = Eigen::Rotation2Dd((((((float)rear_pot - REAR_POT_OFFSET) / 4095.0) * M_PI) * 2.0) * -1);

    m_rightActualMsg.data = right_actual_angle.smallestPositiveAngle();
    m_leftActualMsg.data = left_actual_angle.smallestPositiveAngle();
    m_rearActualMsg.data = rear_actual_angle.smallestPositiveAngle();
}

MotorPowers SwerveController::calculateLeftModule(Eigen::Vector2d target_velocity, double rotation_velocity) {
    MotorPowers motor_powers = leftSwerveModule.InverseKinematics(target_velocity, rotation_velocity, left_actual_angle);

    return motor_powers;
}

MotorPowers SwerveController::calculateRightModule(Eigen::Vector2d target_velocity, double rotation_velocity) {
    right_motor_powers = rightSwerveModule.InverseKinematics(target_velocity, rotation_velocity, right_actual_angle);

    return right_motor_powers;
}

MotorPowers SwerveController::calculateRearModule(Eigen::Vector2d target_velocity, double rotation_velocity) {
    rear_motor_powers = rearSwerveModule.InverseKinematics(target_velocity, rotation_velocity, rear_actual_angle);

    return rear_motor_powers;
}

void SwerveController::periodic() {
    m_rightSetpointMsg.data = rightSwerveModule.getSetpoint().smallestPositiveAngle();
    m_leftSetpointMsg.data = leftSwerveModule.getSetpoint().smallestPositiveAngle();
    m_rearSetpointMsg.data = rearSwerveModule.getSetpoint().smallestPositiveAngle();

    m_rightActualPub->publish(&m_rightActualMsg);
    m_leftActualPub->publish(&m_leftActualMsg);
    // m_rearActualPub->publish(&m_rearActualMsg);

    m_rightSetpointPub->publish(&m_rightSetpointMsg);
    m_leftSetpointPub->publish(&m_leftSetpointMsg);
    m_rearSetpointPub->publish(&m_rearSetpointMsg);

    if (!PID_recieved) {
        if(Node::m_handle->getParam("swerveTurnPID", SwerveController::m_PIDConstants, 3)) {
            leftSwerveModule.setkP(m_PIDConstants[0]);
            rightSwerveModule.setkP(m_PIDConstants[0]);
            rearSwerveModule.setkP(m_PIDConstants[0]);

            leftSwerveModule.setkI(m_PIDConstants[1]);
            rightSwerveModule.setkI(m_PIDConstants[1]);
            rearSwerveModule.setkI(m_PIDConstants[1]);

            leftSwerveModule.setkD(m_PIDConstants[2]);
            rightSwerveModule.setkD(m_PIDConstants[2]);
            rearSwerveModule.setkD(m_PIDConstants[2]);
            PID_recieved = true;
        }        
    }
}

void SwerveController::initialize() {
    Node::m_handle->advertise(*m_rightSetpointPub);
    Node::m_handle->advertise(*m_leftSetpointPub);
    Node::m_handle->advertise(*m_rearSetpointPub);
    Node::m_handle->advertise(*m_rightActualPub);
    Node::m_handle->advertise(*m_leftActualPub);
    Node::m_handle->advertise(*m_rightActualPub);

    // while(!Node::m_handle->connected()) {
    //     Node::m_handle->spinOnce();
    // }
}
