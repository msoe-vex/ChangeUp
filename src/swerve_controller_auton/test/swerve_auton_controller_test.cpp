#include "swerve_auton_controller_node.cpp"
#include <gtest/gtest.h>

Eigen::Vector2d expected_vector_array[];

Eigen::Vector2d vector_array[] = 

TEST(JsonTest, ArrayComparison) {
    for (auto vector : vector_array) {
        EXPECT_EQ(vector, expected_vector_array)
    }
}