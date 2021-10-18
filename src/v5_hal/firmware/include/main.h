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
#include "nodes/NodeManager.h"
#include "nodes/actuator_nodes/MotorNode.h"
#include "nodes/sensor_nodes/ADIAnalogInNode.h"
#include "nodes/sensor_nodes/ADIDigitalInNode.h"
#include "nodes/sensor_nodes/ADIEncoderNode.h"
#include "nodes/sensor_nodes/ADIGyroNode.h"
#include "nodes/sensor_nodes/ADIUltrasonicNode.h"
#include "nodes/sensor_nodes/InertialSensorNode.h"
#include "nodes/sensor_nodes/ControllerNode.h"
#include "nodes/system_nodes/CompetitionStatusNode.h"
#include "nodes/system_nodes/ProsTimeNode.h"
#include "nodes/system_nodes/BatteryNode.h"
#include "nodes/sensor_nodes/InertialSensorNode.h"
#include "nodes/subsystems/drivetrain_nodes/SwerveDriveNode.h"
#include "nodes/system_nodes/ConnectionCheckerNode.h"
#include "nodes/subsystems/drivetrain_nodes/TankDriveNode.h"
#include "nodes/subsystems/drivetrain_nodes/HolonomicDriveNode.h"
#include "nodes/subsystems/ConveyorNode.h"
#include "nodes/subsystems/IntakeNode.h"
#include "nodes/auton_nodes/AutonManagerNode.h"
#include "auton/auton_actions/DeployAction.h"
#include "auton/auton_actions/DriveStraightAction.h"
#include "auton/auton_actions/IntakeAction.h"
#include "auton/auton_actions/UpdateConveyorStateAction.h"
#include "auton/auton_actions/FollowPathAction.h"
#include "auton/auton_routines/ProgrammingSkillsAuton.h"
#include "eigen/Eigen/Dense"
#include "pathing/PathManager.h"
#include "util/Logger.h"

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
