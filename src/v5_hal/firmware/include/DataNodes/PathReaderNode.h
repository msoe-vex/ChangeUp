#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/path_reader/Auton.h"
#include "ros_lib/path_reader/Path.h"
#include "ros_lib/path_reader/Waypoint.h"

class PathReaderNode : public Node {
private:
    std::string m_handle_name;
    std::string m_sub_publish_data_name;
    ros::Subscriber<path_reader::Auton, PathReaderNode>* m_move_motor_voltage_sub;

    void m_HandleAutonPath(const path_reader::Auton& msg);

public:
    PathReaderNode(NodeManager* node_manager, std::string handle_name);

    void initialize();

    void periodic();

    ~PathReaderNode();
};
