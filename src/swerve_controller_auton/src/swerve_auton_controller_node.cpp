#include "swerve_auton_controller_node.h"

bool SwerveAutonControllerNode::LoadPathsText(string text) {
    json loadedJson;

    try {
        loadedJson = json::parse(text);
    } catch (const exception& e) {
        cout << ("Could not parse paths file:" + string(e.what()));
        return false;
    }

    return LoadPaths(loadedJson);
}

bool SwerveAutonControllerNode::LoadPaths(json loadedJson, vector<Eigen::Vector2d>* pos_vectors) {

    try {
        json pathJson;
        for (auto path : loadedJson["paths"]) {
            string name = path["name"];
            //vector<Waypoint> waypoints;
            for (auto point : path["points"]) {
                //double speed = point["speed"];
                *pos_vectors.emplace_back(Eigen::Vector2d((double)point["y"], (double)point["x"]));
                //Waypoint waypoint = Waypoint(pos, speed, "");
                //waypoints.push_back(waypoint);
            }
            //Path newPath(waypoints);
            //m_paths[name] = newPath;
        }
    } catch (const exception& e) {
        cout << ("Error reading json path! " + string(e.what()));
        return false;
    }

    return true;
}