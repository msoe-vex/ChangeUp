#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/ADIEncoderData.h"

class ADIEncoderNode : public Node {
   private:
    pros::ADIEncoder m_encoder;
    v5_hal::ADIEncoderData m_encoder_msg;
    ros::Publisher m_publisher;
    std::string* m_handle_name;

    void populateMessage();

   public:
    ADIEncoderNode(NodeManager* nodeManager, int port_top, int port_bottom,
                   bool reverse, std::string* handleName);

    void initialize();

    void periodic();

    ~ADIEncoderNode();
};
