//
// Created by Jonathan T. Phung on 3/4/2021.
//

#ifndef CHANGEUP_PID_H
#define CHANGEUP_PID_H

#include <ctime>

class PID {
   private:
    double kP;
    double kI;
    double kD;
    double error;
    clock_t timer;

   public:
    PID(double kP, double kI, double kD);

    calculate()

    ~PID();
};

#endif  // CHANGEUP_PID_H
