#ifndef _ROSSERIAL_VEX_V5_V5_PUBLISHER_H_
#define _ROSSERIAL_VEX_V5_V5_PUBLISHER_H_

#include "main.h"
#include "constants.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/Int16.h"

#include <iterator>
#include <map>

template <typename T>
class V5Publisher {
protected:
    std::map<ros::Publisher*, T*> _messages;
public:
    void addMessageHolder(ros::Publisher* pub, T* msg) {
        _messages.insert(std::pair<ros::Publisher*, T*>(pub, msg));
    };

    void publish(ros::NodeHandle& handle) {
        typename std::map<ros::Publisher*, T*>::iterator msg_itr;

        for (msg_itr = _messages.begin(); msg_itr != _messages.end(); msg_itr++) {
            ros::Publisher* pub = msg_itr->first;
            T* msg = msg_itr->second;

            pub->publish(msg);
        }

        handle.spinOnce();
    };
};

#endif