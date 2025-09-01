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

#ifndef ROMEA_PATH_FOLLOWING__LONGITUDINAL_CONTROL__CONSTANT_HPP_
#define ROMEA_PATH_FOLLOWING__LONGITUDINAL_CONTROL__CONSTANT_HPP_

// std
#include <string>

// romea
#include <romea_core_path_following/longitudinal_control/constant.hpp>

namespace romea::ros2::path_following
{

template<typename CommandType>
class LongitudinalControlConstant
: public core::path_following::LongitudinalControlConstant<CommandType>
{
public:
  using Core = core::path_following::LongitudinalControlConstant<CommandType>;
  using Parameters = typename Core::Parameters;

public:
  template<typename Node>
  LongitudinalControlConstant(
    std::shared_ptr<Node> /*node*/, const std::string & /*ns*/ = "longitudinal_control")
  : Core({})
  {
  }
};

}  // namespace romea::ros2::path_following

#endif  // ROMEA_PATH_FOLLOWING__LONGITUDINAL_CONTROL__CONSTANT_HPP_
