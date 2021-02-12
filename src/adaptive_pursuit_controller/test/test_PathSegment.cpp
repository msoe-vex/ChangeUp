#include "gtest/gtest.h"
#include "PathSegment.h"

//TEST(PathSegmentTest, FindsStartPoint) {
//    Waypoint startPoint(Vector2d(0, 0), 0);
//    Waypoint endPoint(Vector2d(0, 10), 5);
//
//    PathSegment segment(startPoint, endPoint);
//
//    EXPECT_EQ(segment.getLength(), 10);
//    EXPECT_EQ(segment.getSpeed(0), 0);
//    EXPECT_EQ(segment.getSpeed(10), 5);
//    EXPECT_EQ(segment.getSpeed(5), 2.5);
//
//    Vector2d intersection = segment.getCircularIntersectionPoint(Vector2d(5, 5), 5);
//
//    EXPECT_EQ(intersection.x(), 0);
//    EXPECT_EQ(intersection.y(), 5);
//
//    auto report = segment.getClosestPoint(Vector2d(0, 0), 0);
//    EXPECT_EQ(report.closestPoint.x(), 0);
//    EXPECT_EQ(report.closestPoint.y(), 0);
//    EXPECT_EQ(report.distanceToStart, 0);
//    EXPECT_EQ(report.distanceToEnd, 10);
//    EXPECT_EQ(report.distanceAway, 0);
//    EXPECT_EQ(report.speed, 0);
//
//    report = segment.getClosestPoint(Vector2d(0, 0), 5);
//    EXPECT_EQ(report.closestPoint.x(), 0);
//    EXPECT_EQ(report.closestPoint.y(), 5);
//    EXPECT_EQ(report.distanceToStart, 0);
//    EXPECT_EQ(report.distanceToEnd, 5);
//    EXPECT_EQ(report.distanceAway, 5);
//    EXPECT_EQ(report.speed, 2.5);
//
//    report = segment.getClosestPoint(Vector2d(1, 2), 2);
//    EXPECT_EQ(report.closestPoint.x(), 0);
//    EXPECT_EQ(report.closestPoint.y(), 2);
//    EXPECT_EQ(report.distanceToStart, 0);
//    EXPECT_EQ(report.distanceToEnd, 8);
//    EXPECT_EQ(report.distanceAway, 1);
//    EXPECT_EQ(report.speed, 1);
//}
