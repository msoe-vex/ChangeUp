/*
#include "gtest/gtest.h"
#include "PathManager.h"


TEST(PathTest, CanLoadFromText) {
    auto sucess = PathManager::GetInstance()->LoadPathsText("{\n"
                          "    \"sharedWaypoints\": [],\n"
                          "    \"paths\": [\n"
                          "        {\n"
                          "            \"name\": \"FirstPath\",\n"
                          "            \"waypoints\": [\n"
                          "                {\n"
                          "                    \"name\": \"startWaypoint\",\n"
                          "                    \"angle\": 0,\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 7.5,\n"
                          "                    \"shared\": false\n"
                          "                },\n"
                          "                {\n"
                          "                    \"name\": \"endWaypoint\",\n"
                          "                    \"angle\": 0,\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 71,\n"
                          "                    \"shared\": false\n"
                          "                }\n"
                          "            ],\n"
                          "            \"points\": [\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 7.5,\n"
                          "                    \"speed\": 0\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 11.47,\n"
                          "                    \"speed\": 25.2\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 15.44,\n"
                          "                    \"speed\": 35.64\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 19.41,\n"
                          "                    \"speed\": 43.65\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 23.38,\n"
                          "                    \"speed\": 50.4\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 27.34,\n"
                          "                    \"speed\": 56.34\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 31.31,\n"
                          "                    \"speed\": 61.72\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 35.28,\n"
                          "                    \"speed\": 66.67\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 39.25,\n"
                          "                    \"speed\": 71.27\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 43.22,\n"
                          "                    \"speed\": 66.67\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 47.19,\n"
                          "                    \"speed\": 61.72\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 51.16,\n"
                          "                    \"speed\": 56.34\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 55.13,\n"
                          "                    \"speed\": 50.39\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 59.09,\n"
                          "                    \"speed\": 43.65\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 63.06,\n"
                          "                    \"speed\": 35.64\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 67.03,\n"
                          "                    \"speed\": 25.2\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 0,\n"
                          "                    \"y\": 71,\n"
                          "                    \"speed\": 0\n"
                          "                }\n"
                          "            ]\n"
                          "        },\n"
                          "        {\n"
                          "            \"name\": \"TestPath\",\n"
                          "            \"waypoints\": [\n"
                          "                {\n"
                          "                    \"name\": \"start\",\n"
                          "                    \"angle\": 0,\n"
                          "                    \"x\": 20,\n"
                          "                    \"y\": 10,\n"
                          "                    \"shared\": false\n"
                          "                },\n"
                          "                {\n"
                          "                    \"name\": \"mid\",\n"
                          "                    \"angle\": 0,\n"
                          "                    \"x\": 45,\n"
                          "                    \"y\": 30,\n"
                          "                    \"shared\": false\n"
                          "                },\n"
                          "                {\n"
                          "                    \"name\": \"end\",\n"
                          "                    \"angle\": 0,\n"
                          "                    \"x\": 30,\n"
                          "                    \"y\": 70,\n"
                          "                    \"shared\": false\n"
                          "                }\n"
                          "            ],\n"
                          "            \"points\": [\n"
                          "                {\n"
                          "                    \"x\": 20,\n"
                          "                    \"y\": 10,\n"
                          "                    \"speed\": 0\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 20.58,\n"
                          "                    \"y\": 13.93,\n"
                          "                    \"speed\": 9.02\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 22.18,\n"
                          "                    \"y\": 16.59,\n"
                          "                    \"speed\": 7.31\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 24.56,\n"
                          "                    \"y\": 18.27,\n"
                          "                    \"speed\": 10.17\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 27.51,\n"
                          "                    \"y\": 19.24,\n"
                          "                    \"speed\": 21.63\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 30.8,\n"
                          "                    \"y\": 19.8,\n"
                          "                    \"speed\": 31.65\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 34.2,\n"
                          "                    \"y\": 20.2,\n"
                          "                    \"speed\": 31.65\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 37.49,\n"
                          "                    \"y\": 20.76,\n"
                          "                    \"speed\": 21.63\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 40.44,\n"
                          "                    \"y\": 21.73,\n"
                          "                    \"speed\": 10.17\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 42.82,\n"
                          "                    \"y\": 23.41,\n"
                          "                    \"speed\": 7.31\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 44.42,\n"
                          "                    \"y\": 26.07,\n"
                          "                    \"speed\": 9.02\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 45,\n"
                          "                    \"y\": 30,\n"
                          "                    \"speed\": 26.78\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 45,\n"
                          "                    \"y\": 30,\n"
                          "                    \"speed\": 26.78\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 44.7,\n"
                          "                    \"y\": 33.69,\n"
                          "                    \"speed\": 24.95\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 43.89,\n"
                          "                    \"y\": 37.19,\n"
                          "                    \"speed\": 28.53\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 42.66,\n"
                          "                    \"y\": 40.53,\n"
                          "                    \"speed\": 37.19\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 41.11,\n"
                          "                    \"y\": 43.75,\n"
                          "                    \"speed\": 44.21\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 39.36,\n"
                          "                    \"y\": 46.89,\n"
                          "                    \"speed\": 50.3\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 37.5,\n"
                          "                    \"y\": 50,\n"
                          "                    \"speed\": 55.77\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 35.64,\n"
                          "                    \"y\": 53.11,\n"
                          "                    \"speed\": 50.3\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 33.89,\n"
                          "                    \"y\": 56.25,\n"
                          "                    \"speed\": 44.21\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 32.34,\n"
                          "                    \"y\": 59.47,\n"
                          "                    \"speed\": 37.19\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 31.11,\n"
                          "                    \"y\": 62.81,\n"
                          "                    \"speed\": 28.53\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 30.3,\n"
                          "                    \"y\": 66.31,\n"
                          "                    \"speed\": 24.34\n"
                          "                },\n"
                          "                {\n"
                          "                    \"x\": 30,\n"
                          "                    \"y\": 70,\n"
                          "                    \"speed\": 0\n"
                          "                }\n"
                          "            ]\n"
                          "        }\n"
                          "    ]\n"
                          "}");
    EXPECT_TRUE(sucess);

    auto paths = PathManager::GetInstance()->GetPaths();

    auto waypoints = paths[0].getWaypoints();

    EXPECT_EQ(waypoints[0].getPoint().x(), 0);
    EXPECT_EQ(waypoints[0].getPoint().y(), 7.5);
    EXPECT_EQ(waypoints[1].getPoint().y(), 11.47);
}

TEST(PathTest, CanLoadFromFile) {
    auto sucess = PathManager::GetInstance()->LoadPathsFile("test/testPaths/simplePath.json");
    EXPECT_TRUE(sucess);

    auto paths = PathManager::GetInstance()->GetPaths();

    auto waypoints = paths[0].getWaypoints();

    EXPECT_EQ(waypoints[0].getPoint().x(), 0);
    EXPECT_EQ(waypoints[0].getPoint().y(), 7.5);
    EXPECT_EQ(waypoints[1].getPoint().y(), 11.47);
}

TEST(PathTest, SpeedIsSmooth) {
    Waypoint point1(Vector2d(0, 0), 0);
    Waypoint point2(Vector2d(0, 10), 10);
    Waypoint point3(Vector2d(0, 20), 0);

    vector<Waypoint> waypoints;
    waypoints.push_back(point1);
    waypoints.push_back(point2);
    waypoints.push_back(point3);

    Path path("test", waypoints);

    auto report = path.update(Vector2d(0,0));

    EXPECT_EQ(report.speed, 0);
    EXPECT_EQ(report.distanceAway, 0);
    EXPECT_EQ(report.closestPoint.x(), 0);
    EXPECT_EQ(report.closestPoint.y(), 0);

    report = path.update(Vector2d(0, 5));

    EXPECT_EQ(report.speed, 5);
    EXPECT_EQ(report.distanceAway, 0);
    EXPECT_EQ(report.closestPoint.x(), 0);
    EXPECT_EQ(report.closestPoint.y(), 5);

    report = path.update(Vector2d(0, 10));

    EXPECT_EQ(report.speed, 10);
    EXPECT_EQ(report.distanceAway, 0);
    EXPECT_EQ(report.closestPoint.x(), 0);
    EXPECT_EQ(report.closestPoint.y(), 10);

    report = path.update(Vector2d(0, 15));

    EXPECT_EQ(report.speed, 5);
    EXPECT_EQ(report.distanceAway, 0);
    EXPECT_EQ(report.closestPoint.x(), 0);
    EXPECT_EQ(report.closestPoint.y(), 15);

}
*/
