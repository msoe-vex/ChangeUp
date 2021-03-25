#include "pathing/Path.h"

Path::Path() : m_is_complete(false) {
    
}

Path::Path(vector<PathPoint> pathPoints) : 
        m_pathPoints(pathPoints),
        m_is_complete(false),
        m_last_point(pathPoints.back()) {
}

Pose Path::update(float time) {
    // Remove any points that have been passed
    for (auto it = m_pathPoints.begin(); it != m_pathPoints.end(); it++) {
        if(time > (it + 1)->getTime()) { // Point has been passed
            m_pathPoints.erase(it);
        } else { // Point not yet reached 
            break;
        }
    }

    Pose next_pose;

    if (m_pathPoints.size() == 0) {
        next_pose = m_last_point.getPose();
        m_is_complete = true;
    } else {
        //Interpolate between the first point and the next point
        auto path_point = m_pathPoints.begin()->interpolateTo(*(m_pathPoints.begin() + 1), time);
        next_pose = path_point.getPose();
    }

    return next_pose;
}

vector<PathPoint> Path::getPathPoints() {
    return m_pathPoints;
}

bool Path::isComplete() {
    return m_is_complete;
}