#include <ros/ros.h>
#include <ros/console.h>
#include "geometry_msgs/Vector3.h"
#include "tf/transform_datatypes.h"
#include "Eigen/Dense"
#include "sensor_msgs/Imu.h"

Eigen::Vector2d joystick_target_vector;
geometry_msgs::Vector3 robot_target_vector;
Eigen::Rotation2Dd robot_angle;

void navxDataCallback(const sensor_msgs::Imu& msg) {
	robot_angle = Eigen::Rotation2Dd (tf::getYaw(msg.orientation));
}

void joystickVectorCallback(const geometry_msgs::Vector3& vector) {
	joystick_target_vector(0) = vector.x;
	joystick_target_vector(1) = vector.y;
	Eigen::Vector2d eigen_target_vector = robot_angle.inverse() * joystick_target_vector;
	robot_target_vector.x = eigen_target_vector(0);
	robot_target_vector.y = eigen_target_vector(1);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "fieldOrientedNode");

	ros::NodeHandle handle;

	// Create subscribers and link to callbacks
	ros::Subscriber joystick_vector_sub = handle.subscribe("/swerveCommandJoystick", 100, joystickVectorCallback);
  
	ros::Subscriber navx_data_sub = handle.subscribe("/navx/data", 100, navxDataCallback);

	// Create publishers
	ros::Publisher swerve_command_tf_pub = handle.advertise<geometry_msgs::Vector3>("swerveCommandTf", 100);
	
	ros::Rate loop_rate(50);

	while (ros::ok())
	{
		swerve_command_tf_pub.publish(robot_target_vector);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}