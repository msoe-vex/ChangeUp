#include <ros/ros.h>
#include <Eigen/Dense>
#include "geometry_msgs/Transform.h"


void processCallback(const geometry_msgs::Transform::ConstPtr& msg) {


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "swerve_controller");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("swerve_command_tf", 10, processCallback);

}