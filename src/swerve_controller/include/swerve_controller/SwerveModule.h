#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>

struct motorVectors {
    Eigen::Vector2d motor1Vector;
    Eigen::Vector2d motor2Vector;
};

class SwerveModule {
private:
    Eigen::Vector2d m_moduleLocation; 
    Eigen::Rotation2Dd m_moduleActualAngle;
    double m_rotationAngleThreshold;
    double m_maxRotationSpeed;
    double m_maxVelocity;
    Eigen::Vector2d m_maxMotor1Vector;
    Eigen::Vector2d m_maxMotor2Vector;

public:
    SwerveModule (Eigen::Vector2d moduleLocation, double rotationAngleThreshold, double maxRotationSpeed, double maxVelocity, 
        Eigen::Vector2d maxMotor1Vector, Eigen::Vector2d maxMotor2Vector);

    motorVectors InverseKinematics (Eigen::Vector2d targetVelocity, double targetRotationVelocity);

    Eigen::Rotation2Dd setActualAngle();
};