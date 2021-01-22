#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/Int32.h"

class SingleWireEncoder : public Node {
private:
    std_msgs::Int32 m_encoder_msg;
    std::string m_handle_name;
    ros::Publisher* m_publisher;
    bool m_reversed;

    static volatile int m_ticks;

    void m_encoderHandler(int pin);

    void m_initEncoder();

    int m_getEncoderTicks();

    void m_resetEncoder();

    void m_populateMessage();

public:
    SingleWireEncoder(NodeManager* node_manager, int port,
        std::string handle_name, bool reverse=false);

    void initialize();

    void periodic();

    ~SingleWireEncoder();
};
