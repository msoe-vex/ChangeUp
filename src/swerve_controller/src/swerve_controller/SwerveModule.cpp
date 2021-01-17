#include "swerve_controller/SwerveModule.h"
#include <iostream>

SwerveModule::SwerveModule(Eigen::Vector2d moduleLocation, double rotationAngleThreshold, double maxVelocity, double maxRotationVelocity
    /*Eigen::Vector2d maxMotor1Vector, Eigen::Vector2d maxMotor2Vector*/) {
    m_moduleLocation = moduleLocation;
    m_rotationAngleThreshold = rotationAngleThreshold;
    m_maxVelocity = maxVelocity;
    m_maxRotationVelocity = maxRotationVelocity;
    /*m_maxMotor1Vector = maxMotor1Vector;
    m_maxMotor2Vector = maxMotor2Vector;*/
}

motorMagnitudes* SwerveModule::InverseKinematics (Eigen::Vector2d targetVelocity, double targetRotationVelocity, Eigen::Rotation2Dd moduleActualAngle) {
    if ((targetVelocity(0) == 0) && (targetVelocity(1) && (targetRotationVelocity == 0))) {
        double scaledMotor1Mag = 0;
        double scaledMotor2Mag = 0;
        
        motorMagnitudes* MotorMagnitudes = new motorMagnitudes;
        MotorMagnitudes->motor1Magnitude = scaledMotor1Mag;
        MotorMagnitudes->motor2Magnitude = scaledMotor2Mag;

        return MotorMagnitudes;
    }
    
    Eigen::Vector2d maxMotor1Vector (m_maxVelocity, 100);
    Eigen::Vector2d maxMotor2Vector (-1 * m_maxVelocity, 100);

    float maxMotor1Mag = maxMotor1Vector.norm() / 2;
    float maxMotor2Mag = maxMotor2Vector.norm() / 2;

    std::cout << "\n********************************\n";
    std::cout << "Max Motor Vector Magnitudes: \n";
    std::cout << "maxMotor1Vector: " << maxMotor1Vector.norm();
    std::cout << "\nmaxMotor2Vector: " << maxMotor2Vector.norm();
    std::cout << "\n********************************\n";   

    Eigen::Vector2d rotatedModuleLocation = Eigen::Rotation2Dd (M_PI/2) * m_moduleLocation;
    Eigen::Vector2d targetRotationVector = targetRotationVelocity * rotatedModuleLocation;
    Eigen::Vector2d targetVector = targetVelocity + targetRotationVector;
    std::cout << "\n********************************\n";
    std::cout << "targetVector: " << targetVector;
    std::cout << "\n********************************\n";

    Eigen::Rotation2Dd targetVectorAngle = Eigen::Rotation2Dd (atan2(targetVector(1), targetVector(0)));
    double moduleRotationDelta = (targetVectorAngle * moduleActualAngle.inverse()).smallestAngle(); 
    
    std::cout << "\n********************************\n";
    std::cout << "moduleRotationDelta: " << moduleActualAngle.angle();
    std::cout << "\ntargetVectorAngle: " << (Eigen::Rotation2Dd(0) * moduleActualAngle.inverse()).angle();
    std::cout << "\n********************************\n";

    Eigen::Vector2d motorPowerVector;

    if (moduleRotationDelta >= m_rotationAngleThreshold) {
        motorPowerVector(1) = m_maxRotationVelocity;
    }
    else {
        motorPowerVector(1) = m_maxRotationVelocity * (moduleRotationDelta / m_rotationAngleThreshold);
    }

    motorPowerVector(0) = targetVector.norm();

    std::cout << "\n********************************\n";
    std::cout << "motorPowerVector: " << motorPowerVector;
    std::cout << "\n" << moduleRotationDelta;
    std::cout << "\n********************************\n";
    

    double scaledMotor1Mag = motorPowerVector.dot(maxMotor1Vector) / maxMotor1Mag;
    double scaledMotor2Mag = motorPowerVector.dot(maxMotor2Vector) / maxMotor2Mag;

    std::cout << "\n********************************\n";
    std::cout << "Dot products: \n";
    std::cout << "scaledMotor1Mag: " << motorPowerVector.dot(maxMotor1Vector);
    std::cout << "\nscaledMotor2Mag: " << motorPowerVector.dot(maxMotor2Vector);
    std::cout << "\n********************************\n";    

    std::cout << "\n********************************\n";
    std::cout << "Before Scaling: \n";
    std::cout << "scaledMotor1Mag: " << scaledMotor1Mag;
    std::cout << "\nscaledMotor2Mag: " << scaledMotor2Mag;
    std::cout << "\n********************************\n";

    float motorVectorScalar = 1;

    if (scaledMotor1Mag > (maxMotor1Mag * 2)) {
        if (scaledMotor1Mag > scaledMotor2Mag) {
            motorVectorScalar = (maxMotor1Mag * 2) / scaledMotor1Mag;
        }
        else {
            motorVectorScalar = (maxMotor2Mag * 2) / scaledMotor2Mag;
        }
    }

    scaledMotor1Mag /= sqrt(2);
    scaledMotor2Mag /= sqrt(2);

    scaledMotor1Mag *= motorVectorScalar;
    scaledMotor2Mag *= motorVectorScalar;

    motorMagnitudes* MotorMagnitudes = new motorMagnitudes;

    MotorMagnitudes->motor1Magnitude = scaledMotor1Mag;
    MotorMagnitudes->motor2Magnitude = scaledMotor2Mag;
    
    std::cout << "\n********************************\n";
    std::cout << "scaledMotor1Mag: " << scaledMotor1Mag;
    std::cout << "\nscaledMotor2Mag: " << scaledMotor2Mag;
    std::cout << "\n********************************\n";

    return MotorMagnitudes;
}