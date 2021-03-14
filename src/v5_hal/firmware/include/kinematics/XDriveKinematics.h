#pragma once

#include "api.h"
#include "util/Constants.h"
#include "nodes/subsystems/drivetrain_nodes/HolonomicDriveNode.h"
#include <algorithm>
#include <initializer_list>

class XDriveKinematics {
private:
public:
    XDriveKinematics();

    HolonomicDriveNode::HolonomicDriveMotorPowers InverseKinematics(float x_velocity, float y_velocity, float theta_velocity);

    ~XDriveKinematics();
};