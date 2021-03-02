#include "PathManager.h"

PathManager* PathManager::m_instance = nullptr;

PathManager * PathManager::GetInstance() {
    if(!m_instance) {
        m_instance = new PathManager;
    }

    return m_instance;
}

int PathManager::NumPaths() {
    return m_paths.size();
}

bool PathManager::LoadPathsText(string text) {
    json loadedJson;

    try {
        loadedJson = json::parse(text);
    } catch (const exception& e) {
        Logger::logError("Could not parse paths file:" + string(e.what()));
        return false;
    }

    return LoadPaths(loadedJson);
}

bool PathManager::LoadPaths(json loadedJson) {
    m_paths.clear();

    try {
        json pathJson;
        for (auto path : loadedJson["paths"]) {
            string name = path["name"];
            vector<Waypoint> waypoints;
            for (auto point : path["points"]) {
                double speed = point["speed"];
                Trans2d pos = Trans2d(-(double)point["y"], (double)point["x"]);
                Waypoint waypoint = Waypoint(pos, speed, "");
                waypoints.push_back(waypoint);
            }
            Path newPath(waypoints);
            m_paths[name] = newPath;
        }
    } catch (const exception& e) {
        Logger::logError("Error reading json path! " + string(e.what()));
        return false;
    }

    return true;
}

bool PathManager::LoadPathsFile(string filePath) {
    ifstream pathsFile;
    try {
        pathsFile.open(filePath);
        if(!pathsFile.is_open()) {
            Logger::logError("Could not open paths file at " + filePath);
            return false;
        }
    } catch (const exception& e) {
        Logger::logError("Could not open paths file at " + filePath + " : " + string(e.what()));
        return false;
    }

    json loadedJson;
    try {
        pathsFile >> loadedJson;
    } catch (const exception& e) {
        Logger::logError("Could not parse paths file:" + string(e.what()));
        pathsFile.close();
        return false;
    }

    pathsFile.close();
    return LoadPaths(loadedJson);
}

unordered_map<string, Path> PathManager::GetPaths() {
    return m_paths;
}

Path PathManager::GetPath(string name) {
    if (m_paths.find(name) == m_paths.end()) {
        Logger::logError("Path with key: " + name + " not found!");
        return m_paths[m_paths.begin()->first];
    } else {
        return m_paths[name];
    }   
}
