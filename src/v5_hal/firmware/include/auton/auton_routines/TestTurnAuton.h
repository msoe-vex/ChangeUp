#include "auton/Auton.h"
#include "nodes/subsystems/drivetrain_nodes/TankDriveNode.h"
#include "nodes/sensor_nodes/InertialSensorNode.h"
#include "auton/auton_actions/TurnToAngleAction.h"
#include "eigen/Eigen/Dense"

class TestTurnAuton : public Auton {
public:
    TestTurnAuton(TankDriveNode* chassis_node, InertialSensorNode* inertial_sensor_node);

    void AddNodes();

private:
    TankDriveNode* m_chassis_node;
    InertialSensorNode* m_inertial_sensor_node;
    AutonNode* m_turn_node;
};