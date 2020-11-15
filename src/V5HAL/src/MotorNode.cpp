#include "MotorNode.h"

MotorNode::MotorNode(NodeManager* nodeManager, int portNumber, std::string handle, 
    pros::motor_gearset_e_t gearset, bool reverse):Node(nodeManager, 200) {
    m_motor = new pros::Motor(portNumber, gearset, reverse);
}

void MotorNode::initialize() {
    
}

void MotorNode::execute() {

}

MotorNode::~MotorNode() {
    delete m_motor;
}