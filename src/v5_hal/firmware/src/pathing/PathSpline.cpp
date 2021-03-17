#include "pathing/PathSpline.h"

PathSpline::PathSpline(Waypoint start_waypoint, Waypoint end_waypoint) : 
        m_start_waypoint(start_waypoint), 
        m_end_waypoint(end_waypoint) {

}

void PathSpline::m_generateSpline() {
    
}

void PathSpline::calculate() {
    // Generate the spline
    // Get the start and end angle of the path

    // Get the distance between the two waypoints

    // Get the change of tangent angle between the last and first point of the path

    // 

    // Calculate waypoints in between the start and end
    // should be length parameterized in order to have a set distance between points

    // https://github.com/msoe-vex/WebDashboard/blob/5c112bcf74bd4b63ad7efa2d1fb9093cc9a7cb3f/www/path.js#L112
    int sample_points = 1;

    do {

    } while (true);


}

float PathSpline::getLength() {
    return m_length_m;
}

float PathSpline::getDuration() {
    return m_duration_ms;
}

Waypoint PathSpline::getWaypointAtPercentage(float percentage) {

}

Waypoint PathSpline::getWaypointAtTime(float time) {

}