#include "Auton.h"
#include "adaptive_pursuit_controller/PathManager.h"
#include "DataNodes/TankDriveNode.h"
#include "DataNodes/OdometryNode.h"
#include "DataNodes/ConveyorNode.h"
#include "Actions/FollowPathAction.h"
#include "Actions/DeployAction.h"

class ProgrammingSkillsAuton : public Auton {
public:
    ProgrammingSkillsAuton(TankDriveNode* tankDriveNode, OdometryNode* odom_node, ConveyorNode* conveyer_node);

    void AddNodes();

private:
    TankDriveNode* m_tankDriveNode;
    AutonNode* testNode;
    OdometryNode* m_odom_node;
    ConveyorNode* m_conveyor_node;
};