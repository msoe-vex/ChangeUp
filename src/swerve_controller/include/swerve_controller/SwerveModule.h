#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>

//left motor -> motor1
//right motor -> motor2
struct motorPowers {
    int8_t left_motor_power;
    int8_t right_motor_power;
};

class SwerveModule {
private:
    Eigen::Vector2d m_moduleLocation; 
    double m_rotationAngleThreshold;
    double m_maxVelocity;
    double m_maxRotationVelocity;

public:
    SwerveModule (Eigen::Vector2d moduleLocation, double rotationAngleThreshold, double maxVelocity, double maxRotationVelocity);

    motorPowers* InverseKinematics (Eigen::Vector2d targetVelocity, double targetRotationVelocity, Eigen::Rotation2Dd moduleActualAngle);

    Eigen::Rotation2Dd setActualAngle();
};