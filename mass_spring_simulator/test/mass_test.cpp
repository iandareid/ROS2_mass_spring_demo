#include <gtest/gtest.h>
#include <rclcpp/utilities.hpp>
#include "simulator.hpp"

TEST(mass_spring, a_first_test)
{
  simulator::Simulator sim = simulator::Simulator(); 

  ASSERT_EQ(4, 2 + 2);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
