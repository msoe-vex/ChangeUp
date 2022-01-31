#pragma once

#include "TankOdometry.h"
#include <optional>
#include "Path.h"
#include "Logger.h"
#include "util/Timer.h"

class VelocityPair {
public:
    VelocityPair(double l, double r);
    double left;
    double right;
    bool end_of_path;
};

class AdaptivePursuit{
private:
    double m_fixedLookahead;
    Path m_path;
    Position2d::Delta m_lastCommand;
    double m_lastTime;
    double m_maxAccel;
    double m_wheelDiameter;
    double m_dt;
    bool m_reversed;
    double m_pathCompletionTolerance;
    bool m_hasRun = false;
    bool m_gradualStop = true;
    Timer m_timer;

public:
    AdaptivePursuit(double fixedLookahead, double maxAccel, double maxDeccel, double nominalDt, Path path,
                    bool reversed, double pathCompletionTolerance, bool gradualStop, double wheelDiameter, Timer timer = Timer());

    bool isDone();

    double getRemainingLength();

    VelocityPair Update(Pose robotPos, double now);

    bool checkEvent(std::string event);

    struct Circle{
        Trans2d center;
        double radius;
        bool turnRight;
        Circle(Trans2d cent, double rad, bool turn_right);
    };

    std::optional<Circle> joinPath(Position2d pos, Trans2d lookahead);
};
