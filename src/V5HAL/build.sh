#!/bin/bash

# Build script for building PROS project
# By default this script will reload the project includes on each build
# Run with -q flag for a quick build not reloading includes

reload_includes() {

  # Build and install catkin workspace
  source /opt/ros/noetic/setup.bash
  cd ../..
  catkin_make install

  # Update includes from ROS project
  source install/setup.bash
  cd src/V5HAL/include
  rosrun rosserial_vex_v5 make_libraries.py .

  # unless the include is a special case
  # (e.g. the include is a std library, or ros_lib is already in the path),
  # add ros_lib to the start of the include.
  grep -irl '#include' ros_lib/ | xargs perl -pi \
    -e 's/^#include "(?!main)(?!pros)(?!stddef)(?!stdlib)(?!string)(?!stdint)(?!math)(?!ros_lib)/#include "ros_lib\//g' \

  cd ..

}

build() {
  prosv5 make
}

while getopts "q" OPTION
do
	case $OPTION in
		q)
      echo -e "\e[32mQuick build - Will not reload ROS includes\n\e[39m"
      build
      exit
			;;
    ?)
     echo "Use flag -q for quick build"
     exit
     ;;
	esac
done


echo -e "\e[32mFull build - Reloading ROS includes\n\e[39m"
reload_includes
build
