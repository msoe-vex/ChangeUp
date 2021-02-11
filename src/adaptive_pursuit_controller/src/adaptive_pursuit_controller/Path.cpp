#include "Path.h"

Waypoint::Waypoint(Trans2d pos, double spd, string completeEvent) {
	position = pos;
	speed = spd;
	event = completeEvent;
}

Path::Path() : Path({Waypoint(Trans2d(), 0)}, false) {

}

Path::Path(vector<Waypoint> waypoints, bool flip) {
	m_waypoints = waypoints;
	for (int i = 0; i < m_waypoints.size() - 1; i++) {
        m_segments.emplace_back(m_waypoints[i].position, m_waypoints[i+1].position,
                                (flip ? m_waypoints[i].rotation.inverse() : m_waypoints[i].rotation),
                                m_waypoints[i].speed);
	}

/*	if(m_waypoints.size() > 0){
		if(m_waypoints[0].event != ""){
			m_events.push_back(m_waypoints[0].event);
		}
		m_waypoints.erase(m_waypoints.begin());
	}*/
}

Path Path::fromFile(string fileName, bool flip) {
    Logger::logInfo("Loading File: " + fileName);
    string fileStarter = "/home/lvuser/Paths/";
    ifstream inFile(fileStarter + fileName);
    if (inFile.is_open()) {
        string text((istreambuf_iterator<char>(inFile)), istreambuf_iterator<char>());
        return fromText(text, flip);
    }
    cout << "Failed to open: " << fileName << endl;
    return Path();
}

Path Path::fromText(string text, bool flip) {
    vector<Waypoint> points;
    json json;
    try {
        json = json::parse(text);
    } catch (const exception& e) {
        Logger::logError("Error parsing json path! " + string(e.what()));
        return Path();
    }

    //Logger::logInfo("Json text contents:\n" + json.dump(4));
    try {
        for (auto point : json) {
            Waypoint waypoint({point["x"].get<double>(), point["y"].get<double>()}, 100);
            if(flip) {
                waypoint.position = waypoint.position.flipX();
            }
            if(point["name"].get<string>() != "point") {
                waypoint.event = point["name"].get<string>();
            }
            points.push_back(waypoint);
        }
    } catch (const exception& e) {
        Logger::logError("Error reading json path! " + string(e.what()));
        return Path();
    }

    if(!points.empty()){
        Logger::logInfo("Path with " + to_string(points.size()) + " points was loaded successfully.");
        return Path(points);
    } else{
        Logger::logError("Loaded path text was empty!");
        return Path();
    }
}

double Path::update(Trans2d pos) {
	double rv = 0.0;
	for(int i = 0; i < m_segments.size(); i++) {
//		PathSegment segment = m_segments[i];
		PathSegment::ClosestPointReport closestPointReport = m_segments[i].getClosestPoint(pos);
//		cout << "Index " << closestPointReport.index << endl;
		if (closestPointReport.index >= .99){
			m_segments.erase(m_segments.begin() + i);
			if(!m_waypoints.empty()){
				if(!m_waypoints[0].event.empty()){
					m_events.push_back(m_waypoints[0].event);
				}
				m_waypoints.erase(m_waypoints.begin());
			}
			return update(pos);
		} else {
			if(closestPointReport.index > 0.0){
				m_segments[i].updateStart(closestPointReport.closestPoint);
			}

			rv = closestPointReport.distance;

			if(m_segments.size() > i + 1){
				PathSegment::ClosestPointReport nextClosestPointReport = m_segments[i+1].getClosestPoint(pos);
				if(nextClosestPointReport.index > 0
						&& nextClosestPointReport.index < .99
						&& nextClosestPointReport.distance < rv){
					m_segments[i+1].updateStart(nextClosestPointReport.closestPoint);
					rv = nextClosestPointReport.distance;
					m_segments.erase(m_segments.begin() + i);
					if(!m_waypoints.empty()){
						if(!m_waypoints[0].event.empty()){
							m_events.push_back(m_waypoints[0].event);
						}
						m_waypoints.erase(m_waypoints.begin());
					}
				}
			}
			break;
		}
	}
	return rv;
}

bool Path::eventPassed(string event) {
	return (find(m_events.begin(), m_events.end(), event) != m_events.end());
}

double Path::getRemainingLength() {
	double length = 0.0;
	for (auto i: m_segments){
		length += i.getLength();
	}
	return length;
}

PathSegment::Sample Path::getLookaheadPoint(Trans2d pos, double lookahead) {
	if(m_segments.empty()){
		return {Trans2d(), 0};
	}

	Trans2d posInverse = pos.inverse();
	if(posInverse.translateBy(m_segments[0].getStart()).norm() >= lookahead){
		return {m_segments[0].getStart(), m_segments[0].getSpeed()};
	}
	for (auto segment : m_segments) {
        double distance = posInverse.translateBy(segment.getEnd()).norm();
		if(distance >= lookahead){
			pair<bool, Trans2d> intersectionPoint =
                    getFirstCircleSegmentIntersection(segment, pos, lookahead);
			if(intersectionPoint.first){
				return {intersectionPoint.second, segment.getSpeed()};
			} else {
				cout << "Error? Bad things happened" << endl;
			}
		}
	}

	PathSegment lastSegment = m_segments[m_segments.size() - 1];
	PathSegment newLastSegment = PathSegment(lastSegment.getStart(), lastSegment.interpolate(10000),
                                             lastSegment.getAngle(), lastSegment.getSpeed());
	pair<bool, Trans2d> intersectionPoint = getFirstCircleSegmentIntersection(newLastSegment, pos,
			lookahead);
	if(intersectionPoint.first){
		return {intersectionPoint.second, lastSegment.getSpeed()};
	} else {
		cout << "Error? REALLY Bad things happened" << endl;
		return {lastSegment.getEnd(), lastSegment.getSpeed()};
	}
}

pair<bool, Trans2d> Path::getFirstCircleSegmentIntersection(PathSegment segment, Trans2d center,
                                                                       double radius) {
	double x1 = segment.getStart().getX() - center.getX();
	double y1 = segment.getStart().getY() - center.getY();
	double x2 = segment.getEnd().getX() - center.getX();
	double y2 = segment.getEnd().getY() - center.getY();
	double dx = x2 - x1;
	double dy = y2 - y1;
	double drSquared = dx * dx + dy * dy;
	double det = x1 * y2 - x2 * y1;

	double discriminant = drSquared *radius * radius - det * det;
	if (discriminant < 0) {
		return {false, Trans2d()};
	}

	double sqrtDiscriminant = sqrt(discriminant);
	Trans2d posSolution = Trans2d(
			(det * dy + ((dy < 0) ? -1 : 1) * dx * sqrtDiscriminant) / drSquared + center.getX(),
			(-det * dx + abs(dy) * sqrtDiscriminant) / drSquared + center.getY());
	Trans2d negSolution = Trans2d(
			(det * dy - ((dy < 0) ? -1 : 1) * dx * sqrtDiscriminant) / drSquared + center.getX(),
			(-det * dx - abs(dy) * sqrtDiscriminant) / drSquared + center.getY());

	double posDot = segment.dotProduct(posSolution);
	double negDot = segment.dotProduct(negSolution);
	if (posDot < 0 && negDot >= 0){
		return {true, negSolution};
	} else if (posDot >= 0 && negDot < 0){
		return {true, posSolution};
	} else {
		if (abs(posDot) <= abs(negDot)){
			return {true, posSolution};
		} else {
			return {true, negSolution};
		}
	}
}

Waypoint Path::getFirstWaypoint() {
    return m_waypoints[0];
}

Position2d Path::getClosestPoint(Trans2d pos) {
    Position2d closestPoint = Position2d(m_segments[0].getStart(), m_segments[0].getAngle());
    double closestDistance = hypot(pos.getX() - closestPoint.getTranslation().getX(),
                                   pos.getY() - closestPoint.getTranslation().getY());
    for (unsigned int i = 1; i < m_segments.size(); i++){
        PathSegment segment = m_segments[i];
        double distance = hypot(pos.getX() - segment.getStart().getX(),
                                pos.getY() - segment.getStart().getY());
        if(distance < closestDistance) {
            closestPoint = Position2d(segment.getStart(), segment.getAngle());
            closestDistance = distance;
        }
    }
    return closestPoint;
}

vector<Waypoint> Path::getWaypoints() {
    return m_waypoints;
}