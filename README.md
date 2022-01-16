# MSOE Robotics ChangeUp Project
![ROS and PROS Build](https://github.com/msoe-vex/ChangeUp/workflows/ROS%20Noetic%20and%20PROS%20Build%20CI/badge.svg)

ROS workspace for the 2020-2021 VEX Game Change Up.

## Cloning the Repository
To build this project with Docker Desktop, autocrlf must be set to false in Git **before cloning**. This can be done with the following command:

`git config --global core.autocrlf false`

This repository contains *submodules*, which are other repositories that have essentially been "merged" in with this project. To properly clone this project, you need 
to also clone the submodules. This can be done with the following command:

**SSH**

`git clone --recurse-submodules git@github.com:msoe-vex/ChangeUp.git`


**HTTPS**

`git clone --recurse-submodules https://github.com/msoe-vex/ChangeUp.git`


## Running Tests

Tests are run through the ROS architecture and can be triggered from the root directory, with the following command:

`catkin_make run_tests`

This will run *all* tests in the project. This searches for tests in all packages, as well as all package dependencies.

To run specific tests, follow the postfix:

`catkin_make run_tests[PACKAGE NAME]`

Examples in this project include:

```
catkin_make run_tests_v5_hal
catkin_make run_tests_rosserial
catkin_make run_tests_navx_publisher
[...]
```

Testing output should print as follows:

```
[----------] Global test environment tear-down
[==========] 10 tests from 1 test suite ran. (6 ms total)
[  PASSED  ] 8 tests.
[  FAILED  ] 2 tests, listed below:
[  FAILED  ] TestSuite.TestForSpecificActionOne
[  FAILED  ] TestSuite.TestForSpecificActionTwo
```

Specific numbers and test names will change, but the structure should be the same. 