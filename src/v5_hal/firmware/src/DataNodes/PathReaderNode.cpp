#include "DataNodes/PathReaderNode.h"

PathReaderNode::PathReaderNode(NodeManager* node_manager, std::string handle_name);

void PathReaderNode::initialize();

void PathReaderNode::periodic();

PathReaderNode::~PathReaderNode();