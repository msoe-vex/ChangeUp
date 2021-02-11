#pragma once

#include "Math/Pose.h"
#include "Math/Math.h"

using namespace std;


class TankOdometry {
public:
    struct EncoderConfig {
        double initialTicks;
        double ticksPerWheelRevolution;
        double wheelDiameter;
    };

    void ResetEncoderTicks(double leftEncoderTicks = 0, double rightEncoderTicks = 0);

    void Update(double leftEncoderRawTicks, double rightEncoderRawTicks, double trackWidth);
    void Update(double leftEncoderRawTicks, double rightEncoderRawTicks, Rotation2Dd gyroAngle);

    static TankOdometry* GetInstance();

    void Initialize(EncoderConfig leftEncoderConfig, EncoderConfig rightEncoderConfig, Pose currentPose = Pose());

    void SetCurrentPose(Pose currentPose);

    Pose GetPose();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    TankOdometry() = default;

    static TankOdometry* m_instance;

    double m_leftTicksToDist;
    double m_rightTicksToDist;

    double m_lastLeftEncoderDist;
    double m_lastRightEncoderDist;

    Rotation2Dd m_gyroInitialAngle;

    bool m_poseReset = true;

    Pose m_robotPose = Pose(Vector2d(0, 0), Rotation2Dd());
};


