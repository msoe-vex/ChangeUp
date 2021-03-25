#include "auton/AutonSequence.h"

AutonSequence::AutonSequence(AutonNode* initialNode) {
    m_auton_sequence = { initialNode, initialNode };
}

void AutonSequence::AddAction(AutonAction* action) {
    m_auton_sequence.tailNode->AddAction(action);
}

void AutonSequence::AddNext(AutonNode* node) {
    m_auton_sequence.tailNode->AddNext(node);
    m_auton_sequence.tailNode = node;
}

AutonSequence::AutonSequenceList AutonSequence::GetSequence() {
    return m_auton_sequence;
}

AutonSequence::~AutonSequence() {
    
}