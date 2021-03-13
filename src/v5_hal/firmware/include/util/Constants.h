#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#define PUBLISH_DELAY_MS 20
#define BALL_PRESENT_THRESHOLD 2700 // Threshold for Indexing Sensor to detect ball
#define MAX_MOTOR_VOLTAGE 12000 // Max voltage value for a motor
#define DELAY_TIME_MILLIS 5 // Standard delay time used in NodeManager
#define GYRO_OFFSET (M_PI_2)
#define MAX_WHEEL_SPEED 1.1526 // Wheel speed for Holonomic Drive in m/s 

// The following was from code that is no longer in current use
#define MAX_ROBOT_SPEED 41.9 // tank drive max robot speed in/s
// Swerve Drive
#define MAX_VELOCITY 1.31 // m/s
#define MAX_ROTATIONAL_VELOCITY 0.17 // rad/s ?
#define ROTATION_ANGLE_THRESHOLD (M_PI / 3) //Keeping this here because it was in the code but never used
#define LEFT_POT_OFFSET 40
#define RIGHT_POT_OFFSET 1714
#define REAR_POT_OFFSET 3083
#define LEFT_MODULE_LOCATION_X -5.25
#define LEFT_MODULE_LOCATION_Y 5.55
#define RIGHT_MODULE_LOCATION_X 5.25
#define RIGHT_MODULE_LOCATION_Y 5.55
#define REAR_MODULE_LOCATION_X 0.0
#define REAR_MODULE_LOCATION_Y -5.21

#endif
