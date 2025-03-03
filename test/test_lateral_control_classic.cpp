// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// std
#include <string>
#include <memory>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "../test/test_helper.h"

#include "romea_path_following/lateral_control/classic.hpp"

class TestLateralControlClassic : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    std::string config_filename = std::string(TEST_DIR) + "/test_lateral_control.yaml";

    rclcpp::NodeOptions no;
    no.arguments({"--ros-args", "--params-file", config_filename});
    node = std::make_shared<rclcpp::Node>("test_lateral_control", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(TestLateralControlClassic, TestOneAxleSteeringGetParameters)
{
  using Command = romea::core::OneAxleSteeringCommand;
  using LateralControl = romea::ros2::path_following::LateralControlClassic<Command>;

  LateralControl::declare_parameters(node, "lateral_control.classic");
  auto parameters = LateralControl::get_parameters(node, "lateral_control.classic");

  EXPECT_DOUBLE_EQ(parameters.gains.front_kd, 10.0);
}

TEST_F(TestLateralControlClassic, TestTwoAxleSteeringGetParameters)
{
  using Command = romea::core::TwoAxleSteeringCommand;
  using LateralControl = romea::ros2::path_following::LateralControlClassic<Command>;

  LateralControl::declare_parameters(node, "lateral_control.classic");
  auto parameters = LateralControl::get_parameters(node, "lateral_control.classic");

  EXPECT_DOUBLE_EQ(parameters.gains.front_kd, 10.0);
  EXPECT_DOUBLE_EQ(parameters.gains.rear_kd, 11.0);
}

TEST_F(TestLateralControlClassic, TestOneAxleSteeringUpdateGains)
{
  using Command = romea::core::OneAxleSteeringCommand;
  using LateralControl = romea::ros2::path_following::LateralControlClassic<Command>;

  LateralControl lateral_control(node, "lateral_control.classic");

  node->set_parameter(rclcpp::Parameter("lateral_control.classic.gains.front_kd", 1.0));
  EXPECT_DOUBLE_EQ(lateral_control.get_gains().front_kd, 1.0);
}

TEST_F(TestLateralControlClassic, TestTwoAxleSteeringUpdateGains)
{
  using Command = romea::core::TwoAxleSteeringCommand;
  using LateralControl = romea::ros2::path_following::LateralControlClassic<Command>;

  LateralControl lateral_control(node, "lateral_control.classic");

  node->set_parameter(rclcpp::Parameter("lateral_control.classic.gains.front_kd", 1.0));
  EXPECT_DOUBLE_EQ(lateral_control.get_gains().front_kd, 1.0);
  node->set_parameter(rclcpp::Parameter("lateral_control.classic.gains.rear_kd", 2.0));
  EXPECT_DOUBLE_EQ(lateral_control.get_gains().rear_kd, 2.0);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
