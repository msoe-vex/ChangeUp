#include "swerve_controller/SwerveModule.h"

SwerveModule::SwerveModule(Eigen::Vector2d moduleLocation, double rotationAngleThreshold, double maxRotationSpeed, double maxVelocity, 
    Eigen::Vector2d maxMotor1Vector, Eigen::Vector2d maxMotor2Vector) {
    m_moduleLocation = moduleLocation;
    m_rotationAngleThreshold = rotationAngleThreshold;
    m_maxRotationSpeed = maxRotationSpeed;
    m_maxVelocity = maxVelocity;
    m_maxMotor1Vector = maxMotor1Vector;
    m_maxMotor2Vector = maxMotor2Vector;
}

motorVectors SwerveModule::InverseKinematics (Eigen::Vector2d targetVelocity, double targetRotationVelocity) {
    Eigen::Vector2d rotatedModuleLocation = Eigen::Rotation2Dd (M_PI/2) * m_moduleLocation;
    Eigen::Vector2d targetRotationVector = targetRotationVelocity * rotatedModuleLocation;
    Eigen::Vector2d targetVector = targetVelocity + targetRotationVector;

    Eigen::Rotation2Dd targetVectorAngle = Eigen::Rotation2Dd (atan2(targetVector(1), targetVector(0)));
    double moduleRotationDelta = (targetVectorAngle * m_moduleActualAngle.inverse()).smallestAngle();

    Eigen::Vector2d motorPowerVector;

    if (moduleRotationDelta >= m_rotationAngleThreshold) {
        motorPowerVector(1) = m_maxRotationSpeed;
    }
    else {
        motorPowerVector(1) = m_maxRotationSpeed * (moduleRotationDelta / m_rotationAngleThreshold);
    }

    motorPowerVector(0) = targetVector.norm() / m_maxVelocity;

    Eigen::Vector2d scaledMotor1Mag = motorPowerVector.dot(m_maxMotor1Vector) / m_maxMotor1Vector.norm();
    Eigen::Vector2d scaledMotor2Mag = motorPowerVector.dot(m_maxMotor2Vector) / m_maxMotor2Vector.norm();

    float motorVectorScalar = 1;

    if (scaledMotor1Mag > m_maxMotor1Vector.norm() {
        if (scaledMotor1Mag > scaledMotor2Mag) {
            motorVectorScalar = m_maxMotor1Vector.norm() / scaledMotor1Mag;
        }
        else {
            motorVectorScalar = m_maxMotor2Vector.norm() / scaledMotor2Mag;
        }
    }
    scaledMotor1Vector *= motorVectorScalar;
    scaledMotor2Vector *= motorVectorScalar;

    motorVectors* MotorVectors = new motorVectors;

    MotorVectors->motor1Vector = scaledMotor1Vector;

    return MotorVectors;
}