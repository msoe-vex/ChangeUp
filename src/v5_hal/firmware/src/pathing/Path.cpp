#include "pathing/Path.h"

Path::Path() {

}

Path::Path(json jsonToLoad) {
    m_pathPoints.clear();
    for (auto point : jsonToLoad["points"]) {
        Vector2d linear_velocity(point["vx"], point["vy"]);
        float time = point["time"];
        float rotational_velocity = point["omega"];
        Rotation2Dd rotation((double)point["theta"]);
        Vector2d position((double)point["x"], (double)point["y"]);
        m_pathPoints.push_back(PathPoint(time, Pose(position, rotation), linear_velocity, rotational_velocity));
    }
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