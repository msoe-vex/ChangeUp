#pragma once

#include "api.h"
#include "NodeManager.h"

class MotorNode : public Node {
private:
    pros::Motor* m_motor;

public:
    MotorNode(NodeManager* nodeManager, int portNumber, std::string handle, 
        pros::motor_gearset_e_t gearset=pros::E_MOTOR_GEARSET_18, bool reverse=false);

    void initialize();

    void execute();

    ~MotorNode();
};
