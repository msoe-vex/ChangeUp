#include "swerve_controller/SwerveModule.h"

SwerveModule::SwerveModule(Eigen::Vector2d moduleLocation, double rotationAngleThreshold, double maxRotationSpeed, double maxVelocity, Eigen::Vector2d maxMotor1Vector) {
    m_moduleLocation = moduleLocation;
    m_rotationAngleThreshold = roationAngleThreshold;
    m_maxRotationSpeed = maxRotationSpeed;
    m_maxVelocity = maxVelocity;
    m_maxMotor1Vector = maxMotor1Vector;
}

void InverseKinematics (Eigen::Vector2d targetVelocity, double targetRotationVelocity) {
    Eigen::Vector2d rotatedModuleLocation = Eigen::Rotation2Dd (M_PI/2) * m_moduleLocation;
    Eigen::Vector2d targetRotationVector = targetRotationVelocity * rotatedModuleLocation;
    Eigen::Vector2d targetVector = targetVelocity + targetRotationVector;

    Eigen::Rotation2Dd targetVectorAngle = Eigen::Rotation2Dd (math.atan2(targetVector(1), targetVector(0)))
    double moduleRotationDelta = (targetVectorAngle * m_moduleActualAngle.inverse()).smallestAngle();

    Eigen::Vector2d motorPowerVector;

    if (moduleRotationDelta >= m_rotationAngleThreshold) {
        motorPowerVector(1) = m_maxRotationSpeed;
    }
    else {
        motorPowerVector(1) = m_maxRotationSpeed * (moduleRotationDelta / m_rotationAngleThreshold);
    }

    motorPowerVector(0) = targetVector.norm() / m_maxVelocity;

    Eigen::Vector2d scaledMotor1Vector = motorPowerVector.dot(m_maxMotor1Vector) / (pow(m_maxMotorVector(0), 2) + pow(m_maxMotorVector(1), 2)) * m_maxMotorVector;
}