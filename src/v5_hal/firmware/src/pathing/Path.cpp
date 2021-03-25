#include "pathing/Path.h"

Path::Path() {
    
}

Path::Path(vector<PathPoint> pathPoints) {
    m_pathPoints = pathPoints;
}

Pose* Path::update(float time) {
    // Remove any points that have been passed
    for (auto it = m_pathPoints.begin(); it != m_pathPoints.end(); it++) {
        if(time > (it + 1)->getTime()) { // Point has been passed
            m_pathPoints.erase(it);
        } else { // Point not yet reached 
            break;
        }
    }

    if (m_pathPoints.size() == 0) {
        return nullptr;
    }

    //Interpolate between the first point and the next point
    auto pathPoint = m_pathPoints.begin()->interpolateTo(*(m_pathPoints.begin() + 1), time);

    Pose* next_pose = new Pose(pathPoint.getPose());
    return next_pose;
}