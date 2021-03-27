#include "auton/AutonSequencePresets.h"

AutonSequence::AutonSequenceList getGoalScoringSequence(AutonNode* initial_node) {
    AutonSequence* goal_scoring_sequence = new AutonSequence(initial_node);

    //goal_scoring_sequence->AddNext(new AutonNode())

    return goal_scoring_sequence->GetSequence();
}