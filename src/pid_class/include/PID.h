//
// Created by Jonathan T. Phung on 3/4/2021.
//

#ifndef CHANGEUP_PID_H
#define CHANGEUP_PID_H

#include "util/Timer.h"

// TODO: Javadoc
/**
 * This class represents the
 *
 * To maximize accuracy in robot position estimation, this function should be called as frequently as possible, ideally
 * at a rate of 50 Hz or greater.
 *
 * @param x_encoder_raw_ticks The distance in ticks measured by the X encoder since when the tracker was last run
 * @param y_encoder_raw_ticks The distance in ticks measured by the Y encoder since when the tracker was last run
 * @param gyro_angle The current yaw of the robot as measured by the gyro
 */

class PID {
   private:
    double kP;
    double kI;
    double kD;
    double previousError;
    double totalError;
    Timer timer;

   public:
    PID(double kP, double kI, double kD);

    double calculate(double power, double feedForward)

    ~PID();
};

#endif  // CHANGEUP_PID_H
