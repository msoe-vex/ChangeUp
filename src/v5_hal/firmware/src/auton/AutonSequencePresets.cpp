#include "auton/AutonSequencePresets.h"

AutonSequence::AutonSequenceList getGoalScoringSequence(AutonNode* initial_node) {
    AutonSequence* goal_scoring_sequence = new AutonSequence(initial_node);

    

    return goal_scoring_sequence->GetSequence();
}