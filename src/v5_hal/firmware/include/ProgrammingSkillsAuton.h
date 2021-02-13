#include "Auton.h"
#include "adaptive_pursuit_controller/PathManager.h"
#include "DataNodes/TankDriveNode.h"
#include "Actions/FollowPathAction.h"

class ProgrammingSkillsAuton : public Auton {
public:
    ProgrammingSkillsAuton(TankDriveNode * tankDriveNode);
    void AddNodes();
private:
    TankDriveNode * m_tankDriveNode;
    AutonNode * testNode;
};