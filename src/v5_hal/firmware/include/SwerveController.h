#include "SwerveModule.h"
#include "eigen/Eigen/Dense"
#include "ros_lib/std_msgs/Float32.h"
#include "NodeManager.h"

class SwerveController : public Node {
    private:
    Eigen::Rotation2Dd left_actual_angle;
    Eigen::Rotation2Dd right_actual_angle;
    Eigen::Rotation2Dd rear_actual_angle;

    SwerveModule leftSwerveModule;
    SwerveModule rightSwerveModule;
    SwerveModule rearSwerveModule;

    MotorPowers left_motor_powers;
    MotorPowers right_motor_powers;
    MotorPowers rear_motor_powers;

    ros::Publisher* m_rightSetpointPub;
    ros::Publisher* m_leftSetpointPub;
    ros::Publisher* m_rearSetpointPub;

    ros::Publisher* m_rightActualPub;
    ros::Publisher* m_leftActualPub;
    ros::Publisher* m_rearActualPub;

    std_msgs::Float32 m_rightSetpointMsg;
    std_msgs::Float32 m_leftSetpointMsg;
    std_msgs::Float32 m_rearSetpointMsg;

    std_msgs::Float32 m_rightActualMsg;
    std_msgs::Float32 m_leftActualMsg;
    std_msgs::Float32 m_rearActualMsg;

    float m_PIDConstants[3];

    bool PID_recieved;

    const int LEFT_POT_OFFSET = 510; // ADI Port F
    const int RIGHT_POT_OFFSET = 1749; // ADI PORT H
    const int REAR_POT_OFFSET = 3119; // ADI Port G

public:
    SwerveController(NodeManager* node_manager, Eigen::Vector2d left_module_location, Eigen::Vector2d right_module_location, Eigen::Vector2d rear_module_location,
    double rotation_angle_threshold, double max_velocity, double max_rotation_velocity, double kP, double kI, double kD);
     
    void assignActualAngle(int left_pot, int right_pot, int rear_pot);

    void initialize();
    
    void periodic();

    MotorPowers calculateLeftModule(Eigen::Vector2d target_velocity, double rotation_velocity);

    MotorPowers calculateRightModule(Eigen::Vector2d target_velocity, double rotation_velocity);

    MotorPowers calculateRearModule(Eigen::Vector2d target_velocity, double rotation_velocity);
};