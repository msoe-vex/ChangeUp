#include "DataNodes/ADIDigitalOutNode.h"

ADIDigitalOutNode::ADIDigitalOutNode (NodeManager* node_manager, std::string handle_name,
    std::uint8_t port, bool initial_state = false) : Node (nodeManager, 20), 
    ADIDigitalOut (port, intial_state) {
    m_handle_name = handle_name;
    }

ADIDigitalOutNode::initialize () {

}

ADIDigitalOutNode::~ADIDigitalOutNode () {
    
}