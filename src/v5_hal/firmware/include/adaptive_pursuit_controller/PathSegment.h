#pragma once

#include "Position2d.h"

class PathSegment{
public:
	struct Sample{
		Trans2d translation;
		double speed;
		Sample(Trans2d newTranslation, double newSpeed);
	};

protected:
	double m_speed;
	Trans2d m_start;
	Trans2d m_end;
	Trans2d m_startToEnd;
	Rotation2d m_angle;
	double m_length;

public:
	struct ClosestPointReport{
		double index;
		double clampedIndex;
		Trans2d closestPoint;
		double distance;
	};

	PathSegment(Trans2d start, Trans2d end, Rotation2d angle, double speed);

	void updateStart(Trans2d newStart);

	double getSpeed();
	Trans2d getStart();
	Trans2d getEnd();
	double getLength();
	Rotation2d getAngle();

	Trans2d interpolate(double index);
	double dotProduct(Trans2d other);
	ClosestPointReport getClosestPoint(Trans2d queryPoint);
};
