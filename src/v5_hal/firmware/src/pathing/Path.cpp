#include "pathing/Path.h"

Path::Path() {

}

Path Path::fromJson() {

    return Path();
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

    //Interpolate between the first point and the next point
    auto pathPoint = m_pathPoints.begin()->interpolateTo(*(m_pathPoints.begin() + 1), time);

    return pathPoint.getPose();
}