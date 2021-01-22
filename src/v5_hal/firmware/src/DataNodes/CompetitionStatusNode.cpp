#include "DataNodes/CompetitionStatusNode.h"

CompetitionStatusNode::CompetitionStatusNode (NodeManager* node_manager, 
    std::string* handle_name) : Node(node_manager, 20) {
    m_handle_name = handle_name->insert(0, "compStatus/");

    m_publisher = new ros::Publisher(m_handle_name.c_str(), &m_comp_status_msg);

    delete handle_name;
}

void CompetitionStatusNode::initialize() {
    Node::m_handle->initNode();
    Node::m_handle->advertise(*m_publisher);
}

void CompetitionStatusNode::periodic() {
    m_populateMessage();
    m_publisher->publish(&m_comp_status_msg);
    Node::m_handle->spinOnce();
}

void CompetitionStatusNode::m_populateMessage() {
    m_comp_status_msg.status = pros::competition::get_status();
    m_comp_status_msg.is_autonomous = pros::competition::is_autonomous();
    m_comp_status_msg.is_comp_connected = pros::competition::is_connected();
    m_comp_status_msg.is_disabled = pros::competition::is_disabled();
}

CompetitionStatusNode::~CompetitionStatusNode () {
    delete m_publisher;
}
