#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>

class SwerveModule {
private:
    Eigen::Vector2d m_moduleLocation; 
    Eigen::Rotation2Dd m_moduleActualAngle;
    double m_rotationAngleThreshold;
    double m_maxRotaionSpeed;
    double m_maxVelocity;

public:
    SwerveModule (Eigen::Vector2d moduleLocation, double rotationAngleThreshold, double maxRotationSpeed, double maxVelocity);

    void InverseKinematics (Eigen::Vector2d targetVelocity, Eigen::Rotation2Dd targetRotationVelocity);

    Eigen::Rotation2Dd setActualAngle();
};