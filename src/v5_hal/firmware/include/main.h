/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2020, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

 /**
  * If defined, some commonly used enums will have preprocessor macros which give
  * a shorter, more convenient naming pattern. If this isn't desired, simply
  * comment the following line out.
  *
  * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
  * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
  * not convienent for most student programmers.
  */
#define PROS_USE_SIMPLE_NAMES

  /**
   * If defined, C++ literals will be available for use. All literals are in the
   * pros::literals namespace.
   *
   * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
   */
#define PROS_USE_LITERALS

#include "api.h"
#include "DataNodes/MotorNode.h"
#include "DataNodes/ADIAnalogInNode.h"
#include "DataNodes/ADIDigitalInNode.h"
#include "DataNodes/ADIEncoderNode.h"
#include "DataNodes/ADIGyroNode.h"
#include "DataNodes/ADIUltrasonicNode.h"
#include "DataNodes/HelloWorldNode.h"
#include "DataNodes/ControllerNode.h"
#include "DataNodes/CompetitionStatusNode.h"
#include "DataNodes/ProsTimeNode.h"
#include "DataNodes/BatteryNode.h"
#include "DataNodes/InertialSensorNode.h"
#include "DataNodes/DriverControlNode.h"
#include "DataNodes/TankDriveNode.h"
#include "DataNodes/ConveyorNode.h"
#include "Actions/DeployAction.h"
#include "Actions/DriveAction.h"
#include "Actions/IntakeAction.h"
#include "Actions/BottomConveyorAction.h"
#include "Actions/FollowPathAction.h"

   /**
    * You should add more #includes here
    */
    //#include "okapi/api.hpp"
    //#include "pros/api_legacy.h"

    /**
     * If you find doing pros::Motor() to be tedious and you'd prefer just to do
     * Motor, you can use the namespace with the following commented out line.
     *
     * IMPORTANT: Only the okapi or pros namespace may be used, not both
     * concurrently! The okapi namespace will export all symbols inside the pros
     * namespace.
     */
     // using namespace pros;
     // using namespace pros::literals;
     // using namespace okapi;

     /**
      * Prototypes for the competition control tasks are redefined here to ensure
      * that they can be called from user code (i.e. calling autonomous from a
      * button press in opcontrol() for testing purposes).
      */
#ifdef __cplusplus
extern "C" {
#endif
    void autonomous(void);
    void initialize(void);
    void disabled(void);
    void competition_initialize(void);
    void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
 //#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
