#pragma once

#include "Auton.h"

class AutonSequence {
public:
    struct AutonSequenceList{
        AutonNode* headNode;
        AutonNode* tailNode;
    };

    AutonSequence(AutonNode* initialNode);

    void AddNext(AutonNode* node);

    void AddAction(AutonAction* action);

    AutonSequenceList GetSequence();
    
    ~AutonSequence();

private:
    AutonSequenceList m_auton_sequence;
};