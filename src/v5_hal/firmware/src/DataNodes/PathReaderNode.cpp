#include "DataNodes/PathReaderNode.h"

PathReaderNode::PathReaderNode(NodeManager* node_manager, std::string handle_name) : Node(node_manager, 10) {
    m_handle_name = handle_name.insert(0, "pathing/");
    m_sub_auton_path = m_handle_name + "/autonPath";

}

void PathReaderNode::m_handleAutonPath(const path_reader::Auton& msg) {
    
}

void PathReaderNode::initialize() {

}

void PathReaderNode::periodic() {

}

PathReaderNode::~PathReaderNode() {

}