#ifndef _NODE_MANAGER_H_
#define _NODE_MANAGER_H_

#include <vector>

#include "api.h"
#include "constants.h"
#include "ros_lib/V5Publisher.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/Int16.h"

class Node {
   public:
    Node(NodeManager * nodeManager, int intervalMilliseconds) {
        nodeManager->addNode(this, intervalMilliseconds);
    }
    virtual ~Node() {}
    virtual void initialize() {}
    virtual void periodic() = 0;
};

class NodeManager {
   private:
    struct NodeStructure {
        Node* node;
        int triggerMillis;
        int lastExecutedMillis;
    };

    std::vector<NodeStructure> m_nodeStructures;

    long unsigned int (*m_getMillis)(void);

   public:
    NodeManager(long unsigned int (*getMilliseconds)(void));

    void addNode(Node* node, int intervalMilliseconds);

    void initialize();

    void execute();

    ~NodeManager();
};

#endif