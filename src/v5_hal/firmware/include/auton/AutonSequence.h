#pragma once

#include "Auton.h"

class AutonSequence{
private:
    struct AutonSequenceList{
        AutonNode* headNode;
        AutonNode* tailNode;
    };
    AutonSequenceList m_auton_sequence;
public:
    AutonSequence(AutonNode* initialNode);
    void AddNext(AutonNode* node);
    void AddAction(AutonAction* action);
    AutonSequenceList GetSequence();
    ~AutonSequence();
};