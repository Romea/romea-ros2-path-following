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

#include "romea_path_following/sliding_observer/extended/cinematic_linear_tangent.hpp"

class TestSlidingObserverExtendedCinematicLinearTangent : public ::testing::Test
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
    std::string config_filename = std::string(TEST_DIR) + "/test_sliding_observer.yaml";

    rclcpp::NodeOptions no;
    no.arguments({"--ros-args", "--params-file", config_filename});
    node = std::make_shared<rclcpp::Node>("test_sliding_observer", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestSlidingObserverExtendedCinematicLinearTangent, TestOneAxleSteeringGetParameters)
{
  using CommandType = romea::core::OneAxleSteeringCommand;
  using SlidingObserver =
    romea::ros2::path_following::SlidingObserverExtendedCinematicLinearTangent<CommandType>;

  SlidingObserver::declare_parameters(node, "sliding_observer.extended_cinematic");
  auto parameters = SlidingObserver::get_parameters(node, "sliding_observer.extended_cinematic");

  EXPECT_DOUBLE_EQ(parameters.lateralDeviationGain, 7.0);
  EXPECT_DOUBLE_EQ(parameters.courseDeviationGain, 8.0);
  EXPECT_DOUBLE_EQ(parameters.lateralDeviationFilterWeight, 0.09);
  EXPECT_DOUBLE_EQ(parameters.courseDeviationFilterWeight, 0.10);
  EXPECT_DOUBLE_EQ(parameters.frontSlidingAngleFilterWeight, 0.11);
  EXPECT_DOUBLE_EQ(parameters.rearSlidingAngleFilterWeight, 0.12);
}

TEST_F(TestSlidingObserverExtendedCinematicLinearTangent, TestOneAxleSteeringInstantiate)
{
  using CommandType = romea::core::OneAxleSteeringCommand;
  using SlidingObserver =
    romea::ros2::path_following::SlidingObserverExtendedCinematicLinearTangent<CommandType>;

  SlidingObserver observer(node, "sliding_observer.extended_cinematic");
}


TEST_F(TestSlidingObserverExtendedCinematicLinearTangent, TestTwoAxleSteeringGetParameters)
{
  using CommandType = romea::core::TwoAxleSteeringCommand;
  using SlidingObserver =
    romea::ros2::path_following::SlidingObserverExtendedCinematicLinearTangent<CommandType>;

  SlidingObserver::declare_parameters(node, "sliding_observer.extended_cinematic");
  auto parameters = SlidingObserver::get_parameters(node, "sliding_observer.extended_cinematic");

  EXPECT_DOUBLE_EQ(parameters.lateralDeviationGain, 7.0);
  EXPECT_DOUBLE_EQ(parameters.courseDeviationGain, 8.0);
  EXPECT_DOUBLE_EQ(parameters.lateralDeviationFilterWeight, 0.09);
  EXPECT_DOUBLE_EQ(parameters.courseDeviationFilterWeight, 0.10);
  EXPECT_DOUBLE_EQ(parameters.frontSlidingAngleFilterWeight, 0.11);
  EXPECT_DOUBLE_EQ(parameters.rearSlidingAngleFilterWeight, 0.12);
}

TEST_F(TestSlidingObserverExtendedCinematicLinearTangent, TestTwoAxleSteeringInstantiate)
{
  using CommandType = romea::core::TwoAxleSteeringCommand;
  using SlidingObserver =
    romea::ros2::path_following::SlidingObserverExtendedCinematicLinearTangent<CommandType>;

  SlidingObserver observer(node, "sliding_observer.extended_cinematic");
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
