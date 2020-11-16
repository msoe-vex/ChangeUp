cmake_minimum_required(VERSION 3.0.2)

project(v5_hal_firmware)

add_custom_target(build
    COMMENT "BUILDING PROS"
)

add_custom_command(TARGET build PRE_BUILD
    COMMENT "COPYING ROS_LIB INTO INCLUDE"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${ROS_LIB_DIR} ${PROJECT_SOURCE_DIR}/include/ros_lib
)

add_custom_command(TARGET build PRE_BUILD
    COMMENT "FIXING ROS_LIB INCLUDE PATHS"
    COMMAND sh ${PROJECT_SOURCE_DIR}/fixIncludes.sh ${PROJECT_SOURCE_DIR}/include/ros_lib
)


add_custom_command(TARGET build PRE_BUILD
    COMMENT "RUNNING PROSV5 BUILD"
    COMMAND prosv5 build
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}

)

add_custom_target(clean_all
    COMMENT "CLEANING PROS"
    COMMAND prosv5 make clean
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

add_custom_command(TARGET clean_all PRE_BUILD
    COMMENT "REMOVING ROS_LIB"
    COMMAND rm -rf ${PROJECT_SOURCE_DIR}/include/ros_lib
)

add_custom_target(upload
    COMMENT "UPLOADING PROS"
    COMMAND prosv5 upload
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)
