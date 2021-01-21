#include "DataNodes/CompetitionStatusNode.h"

CompetitionStatusNode::CompetitionStatusNode (NodeManager* nodeManager, std::string* handleName)
     : Node(nodeManager, 200), m_publisher(handleName->insert(0, "compStatus/").c_str(), &m_comp_status_msg) {
    m_handle_name = handleName;
}

void CompetitionStatusNode::initialize() {
    Node::m_handle->initNode();
    Node::m_handle->advertise(m_publisher);
}

void CompetitionStatusNode::periodic() {
    populateMessage();
    m_publisher.publish(&m_comp_status_msg);
    Node::m_handle->spinOnce();
}

void CompetitionStatusNode::populateMessage() {
    m_comp_status_msg.status = pros::competition::get_status();
    m_comp_status_msg.is_autonomous = pros::competition::is_autonomous();
    m_comp_status_msg.is_comp_connected = pros::competition::is_connected();
    m_comp_status_msg.is_disabled = pros::competition::is_disabled();
}

CompetitionStatusNode::~CompetitionStatusNode () {
    delete m_handle_name;
}
