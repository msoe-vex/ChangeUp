#pragma once

#include "PathSegment.h"
#include "Logger.h"
#include <vector>
#include <algorithm>
#include "json.hpp"

using namespace nlohmann;
using namespace std;

// struct Waypoint {
// 	Trans2d position;
// 	double speed;
//     Rotation2d rotation;
// 	string event;
//     Waypoint(Trans2d pos, double spd = 0.0, string completeEvent = "");
// };

class Path {
protected:
	vector<Waypoint> m_waypoints;
	vector<PathSegment> m_segments;
	vector<string> m_events;
public:
	Path();
	Path(vector<Waypoint> waypoints, bool flip = false);
	static Path fromFile(string fileName, bool flip);
	static Path fromText(string textPath, bool flip);
	Waypoint getFirstWaypoint();
	double update(Trans2d pos);
	bool eventPassed(string event);
	double getRemainingLength();
    vector<Waypoint> getWaypoints();
	PathSegment::Sample getLookaheadPoint(Trans2d pos, double lookahead);
	pair<bool, Trans2d> getFirstCircleSegmentIntersection(PathSegment segment, Trans2d center,
                                                          double radius);
	Position2d getClosestPoint(Trans2d pos);
};
