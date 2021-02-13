#include "Auton.h"
#include "adaptive_pursuit_controller/PathManager.h"
#include "DataNodes/TankDriveNode.h"
#include "DataNodes/OdometryNode.h"
#include "Actions/FollowPathAction.h"

class ProgrammingSkillsAuton : public Auton {
public:
    ProgrammingSkillsAuton(TankDriveNode* tankDriveNode, OdometryNode* odom_node);

    void AddNodes();

private:
    TankDriveNode* m_tankDriveNode;
    AutonNode* testNode;
    OdometryNode* m_odom_node;
};