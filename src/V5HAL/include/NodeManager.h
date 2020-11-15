#ifndef _NODE_MANAGER_H_
#define _NODE_MANAGER_H_

#include "api.h"
#include "constants.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/Int16.h"
#include "ros_lib/V5Publisher.h"

#include <vector>

class NodeManager {
private:
    struct Callback {
        void (*callbackPtr)(void);
        int triggerMillis;
        int lastExecutedMillis;
    };

    std::vector<Callback> _callbacks;
public:
    NodeManager();

    void addCallback(void (*callbackPtr)(void), int triggerMillis);

    void execute(int elapsedMillis);

    ~NodeManager();
};

#endif