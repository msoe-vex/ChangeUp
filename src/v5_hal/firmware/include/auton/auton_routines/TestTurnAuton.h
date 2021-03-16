#include "auton/Auton.h"
#include "nodes/subsystems/drivetrain_nodes/AbstractDriveNode.h"
#include "nodes/sensor_nodes/InertialSensorNode.h"
#include "auton/auton_actions/TurnToAngleAction.h"
#include "eigen/Eigen/Dense"

class TestTurnAuton : public Auton {
public:
    TestTurnAuton(AbstractDriveNode* drive_node, InertialSensorNode* inertial_sensor_node);

    void AddNodes();

private:
    AbstractDriveNode* m_drive_node;
    InertialSensorNode* m_inertial_sensor_node;
    AutonNode* m_turn_node;
};