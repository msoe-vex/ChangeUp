#pragma once

#include <vector>

class Node;

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

class Node {
   public:
    Node(NodeManager * nodeManager, int intervalMilliseconds) {
        nodeManager->addNode(this, intervalMilliseconds);
    }
    virtual ~Node() {}
    virtual void initialize() {}
    virtual void periodic() = 0;
};