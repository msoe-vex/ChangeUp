#include "DataNodes/SingleWireEncoder.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
SingleWireEncoder::SingleWireEncoder(NodeManager* node_manager, int port,
    std::string handle_name, bool reverse) : Node(node_manager, 20) {
    m_handle_name = handle_name.insert(0, "sensor/");
    m_reversed = reverse;

    m_publisher = new ros::Publisher(m_handle_name.c_str(), &m_encoder_msg);
}

void SingleWireEncoder::initialize() {
    // Initialize the handler, and set up data to publish
    Node::m_handle->initNode();
    Node::m_handle->advertise(*m_publisher);
}

void SingleWireEncoder::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    m_populateMessage();
    m_publisher->publish(&m_encoder_msg);
    Node::m_handle->spinOnce();
}

void SingleWireEncoder::m_encoderHandler(int pin) {
    if (m_reversed) {
        m_ticks--;
    } else {
        m_ticks++;
    }
}

// This method gets called in initialize()
void SingleWireEncoder::m_initEncoder() {
	pinMode(1, INPUT);
	ioSetInterrupt(1, INTERRUPT_EDGE_FALLING, encoderHandler);
}

int SingleWireEncoder::m_getEncoderTicks() {
	return m_ticks;
}

void SingleWireEncoder::m_resetEncoder() {
	m_ticks = 0;
}

void SingleWireEncoder::m_populateMessage() {
    m_encoder_msg.data = m_encoder.get_value();
}

SingleWireEncoder::~SingleWireEncoder() { 
    delete m_publisher;
}