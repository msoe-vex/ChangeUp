#include "eigen/Eigen/Dense"
#include "eigen/Eigen/Geometry"
#include "math.h"
#include "api.h"

struct MotorPowers {
    int8_t left_motor_power;
    int8_t right_motor_power;
};

class SwerveModule {
private:
    Eigen::Vector2d m_module_location; 
    double m_rotation_angle_threshold;
    double m_max_velocity;
    double m_max_rotation_velocity;
    double m_percent_error;
    double m_total_error;
    double kP;
    double kI;
    double kD;
    Eigen::Rotation2Dd m_setpoint;


public:
    SwerveModule (Eigen::Vector2d module_location, double rotation_angle_threshold, double max_velocity, double max_rotation_velocity,
        double kP, double kI, double kD);
    
    void setkP(double kP);
    void setkI(double kI);
    void setkD(double kD);

    Eigen::Rotation2Dd getSetpoint();

    MotorPowers InverseKinematics(Eigen::Vector2d target_velocity, double target_rotation_velocity, Eigen::Rotation2Dd module_actual_angle);
};