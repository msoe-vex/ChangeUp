#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"

class OdometryNode : public Node {
private:    
    std::string m_handle_name;

public:
    OdometryNode(NodeManager* node_manager, std::string handle_name);

    void initialize();

    void periodic();

    ~OdometryNode();
};
