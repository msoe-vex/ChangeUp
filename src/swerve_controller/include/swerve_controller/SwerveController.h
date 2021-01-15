#include <Eigen/Dense>
#include "swerve_controller/SwerveModule.h"
#include <vector>

class SwerveController {
public:
    SwerveController(std::vector<SwerveModule> swerveModules);
    void InverseKinematics(Eigen::Vector2d targetVelocity, Eigen::Rotation2Dd targetRotationVelocity);
};