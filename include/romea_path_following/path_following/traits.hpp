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

#ifndef ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_TRAITS_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_TRAITS_HPP_

// romea
#include "romea_core_path_following/path_following.hpp"
#include "romea_path_following/lateral_control/back_stepping.hpp"
#include "romea_path_following/lateral_control/classic.hpp"
#include "romea_path_following/lateral_control/predictive.hpp"
#include "romea_path_following/lateral_control/front_rear_decoupled.hpp"
#include "romea_path_following/lateral_control/skid_sliding.hpp"
#include "romea_path_following/longitudinal_control/classic.hpp"
#include "romea_path_following/sliding_observer/extended/cinematic_linear_tangent.hpp"
#include "romea_path_following/sliding_observer/extended/cinematic_lyapunov.hpp"
#include "romea_path_following/sliding_observer/picard_skid_backstepping.hpp"

namespace romea::ros2::path_following
{

template<typename CommandType>
struct PathFollowingTraits
{
};

template<>
struct PathFollowingTraits<core::OneAxleSteeringCommand>
{
  using Command = core::OneAxleSteeringCommand;
  using PathFollowingBase = core::path_following::PathFollowingBase<Command>;

  struct LongitudinalControl
  {
    using Classic = LongitudinalControlClassic<Command>;
  };

  struct LateralControl
  {
    using Classic = LateralControlClassic<Command>;
    using Predictive = LateralControlPredictive<Command>;
  };

  struct SlidingObserver
  {
    using ExtendedCinematic = SlidingObserverExtendedCinematicLinearTangent<Command>;
    using ExtendedLyapunov = SlidingObserverExtendedCinematicLyapunov<Command>;
  };
};

template<>
struct PathFollowingTraits<core::TwoAxleSteeringCommand>
{
  using Command = core::TwoAxleSteeringCommand;
  using PathFollowingBase = core::path_following::PathFollowingBase<Command>;

  struct LongitudinalControl
  {
    using Classic = LongitudinalControlClassic<Command>;
  };

  struct LateralControl
  {
    using Classic = LateralControlClassic<Command>;
    using Predictive = LateralControlPredictive<Command>;
    using FrontRearDecoupled = LateralControlFrontRearDecoupled<Command>;
  };

  struct SlidingObserver
  {
    using ExtendedCinematic = SlidingObserverExtendedCinematicLinearTangent<Command>;
    using ExtendedLyapunov = SlidingObserverExtendedCinematicLyapunov<Command>;
  };
};

template<>
struct PathFollowingTraits<core::SkidSteeringCommand>
{
  using Command = core::SkidSteeringCommand;
  using PathFollowingBase = core::path_following::PathFollowingBase<Command>;

  struct LongitudinalControl
  {
    using Classic = LongitudinalControlClassic<Command>;
  };

  struct LateralControl
  {
    using BackStepping = LateralControlBackStepping<Command>;
    using SkidSliding = LateralControlSkidSliding<Command>;
  };

  struct SlidingObserver
  {
    using PicardSkidBackstepping = SlidingObserverPicardSkidBackstepping<Command>;
  };
};

}  // namespace romea::ros2::path_following

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_TRAITS_HPP_
