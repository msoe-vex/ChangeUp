#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>

struct motorMagnitudes {
    double motor1Magnitude;
    double motor2Magnitude;
};

class SwerveModule {
private:
    Eigen::Vector2d m_moduleLocation; 
    double m_rotationAngleThreshold;
    double m_maxVelocity;
    double m_maxRotationVelocity;
    /*Eigen::Vector2d m_maxMotor1Vector;
    Eigen::Vector2d m_maxMotor2Vector;*/

public:
    SwerveModule (Eigen::Vector2d moduleLocation, double rotationAngleThreshold, double maxVelocity, double maxRotationVelocity
        /*Eigen::Vector2d maxMotor1Vector, Eigen::Vector2d maxMotor2Vector*/);

    motorMagnitudes* InverseKinematics (Eigen::Vector2d targetVelocity, double targetRotationVelocity, Eigen::Rotation2Dd moduleActualAngle);

    Eigen::Rotation2Dd setActualAngle();
};