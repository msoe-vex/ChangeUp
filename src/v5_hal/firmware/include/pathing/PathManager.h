#pragma once

#include "Path.h"
#include <memory>
#include <unordered_map>
#include <fstream>

class PathManager {
public:
    static PathManager* GetInstance();

    bool LoadPathsText(string text);
    bool LoadPaths(json pathJson);
    bool LoadPathsFile(string filePath);

    int NumPaths();

    unordered_map<string, Path> GetPaths();

    Path GetPath(string name);

private:
    PathManager() = default;
    unordered_map<string, Path> m_paths;
    static PathManager* m_instance;
};