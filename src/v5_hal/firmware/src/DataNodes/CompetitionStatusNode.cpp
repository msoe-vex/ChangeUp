#include "DataNodes/CompetitionStatusNode.h"

CompetitionStatusNode::CompetitionStatusNode (NodeManager* nodeManager, std::string handleName)
     : Node(nodeManager, 200) {
    m_handle = new ros::NodeHandle;
    m_comp_status_msg = new v5_hal::CompetitionStatus;
    m_handle_name = handleName;
}

void CompetitionStatusNode::initialize() {
    m_publisher = new ros::Publisher(m_handle_name.c_str(), m_comp_status_msg);

    m_handle->initNode();
    m_handle->advertise(*m_publisher);
}

void CompetitionStatusNode::periodic() {
    m_populateCompStatusMsg();
    m_publisher->publish(m_comp_status_msg);
    m_handle->spinOnce();
}

void CompetitionStatusNode::m_populateCompStatusMsg() {
    using namespace pros::competition;
    m_comp_status_msg->status = get_status();
    m_comp_status_msg->is_autonomous = is_autonomous();
    m_comp_status_msg->is_comp_connected = is_connected();
    m_comp_status_msg->is_disabled = is_disabled();
}

CompetitionStatusNode::~CompetitionStatusNode () {
    delete m_handle;
    delete m_comp_status_msg;
    delete m_publisher;
}