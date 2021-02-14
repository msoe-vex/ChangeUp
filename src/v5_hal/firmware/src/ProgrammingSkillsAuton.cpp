#include "ProgrammingSkillsAuton.h"

ProgrammingSkillsAuton::ProgrammingSkillsAuton(TankDriveNode* tank_drive_node, OdometryNode* odom_node,
        ConveyorNode* conveyor_node, InertialSensorNode* inertial_sensor_node) : Auton("Programming Skills"), 
        m_tank_drive_node(tank_drive_node), m_odom_node(odom_node), m_inertial_sensor_node(inertial_sensor_node),
        m_conveyor_node(conveyor_node) {
    
}

void ProgrammingSkillsAuton::AddNodes() {
    m_deploy_node = new AutonNode(2, new DeployAction(m_conveyor_node));

    Auton::AddFirstNode(m_deploy_node);

    m_path_1 = new AutonNode(10, new FollowPathAction(m_tank_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Path1")));

    m_deploy_node->AddNext(m_path_1);

    m_wait_to_close_intakes = new AutonNode(5, new WaitAction(4.0));
    m_wait_to_close_intakes->AddAction(new OpenIntakesAction(m_conveyor_node));

    m_close_intakes = new AutonNode(1, new OpenIntakesAction(m_conveyor_node, false));
    m_wait_to_close_intakes->AddNext(m_close_intakes);

    m_deploy_node->AddNext(m_wait_to_close_intakes);

    m_path_1->AddAction(new IntakeAction(m_conveyor_node));
    m_path_1->AddAction(new BottomConveyorAction(m_conveyor_node, false));
    m_path_1->AddAction(new TopConveyorAction(m_conveyor_node, ConveyorNode::HOLDING));

    // Configure the position for the first path
    Waypoint first_waypoint = PathManager::GetInstance()->GetPath("Path1").getFirstWaypoint();
    Vector2d initial_pos(first_waypoint.position.getY(), -first_waypoint.position.getX());
    Rotation2Dd initial_rot(M_PI);
    Pose initial_pose(initial_pos, initial_rot);
    m_odom_node->setCurrentPose(initial_pose);

    m_path_2 = new AutonNode(10, new FollowPathAction(m_tank_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Path2"), true));

    m_path_1->AddNext(m_path_2);
    m_path_2->AddAction(new IntakeAction(m_conveyor_node, 0));
    m_path_2->AddAction(new BottomConveyorAction(m_conveyor_node, false, 0));
}