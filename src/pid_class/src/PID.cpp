//
// Created by Jonathan T. Phung on 3/4/2021.
//

#include "../include/PID.h"
#include "util/Timer.h"

PID::PID(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    previousError = 0;
    totalError = 0;
    timer.Reset();
}

// TODO: How to use timer?
double PID::calculate(double power, double feedForward) {
    const double deltaError = power - previousError;
    const double totalPower;

    previousError = power;
    error += power;
    totalPower = (kP * power) + (kI * error) + (kD * deltaError);

    return std::copysign(min(fabs(totalPower) + feedForward, 1.0), totalPower);
}
