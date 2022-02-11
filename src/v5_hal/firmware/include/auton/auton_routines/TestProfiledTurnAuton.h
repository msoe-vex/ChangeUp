#include "auton/Auton.h"
#include "nodes/subsystems/drivetrain_nodes/IDriveNode.h"
#include "nodes/sensor_nodes/InertialSensorNode.h"
#include "auton/auton_actions/ProfiledTurnAction.h"
#include "util/Constants.h"
#include "eigen/Eigen/Dense"

class TestProfiledTurnAuton : public Auton {
public:
    TestProfiledTurnAuton(IDriveNode* drive_node, InertialSensorNode* inertial_sensor_node);

    void AddNodes() override;

private:
    IDriveNode* m_drive_node;
    InertialSensorNode* m_inertial_sensor_node;
    AutonNode* m_turn_node;
};