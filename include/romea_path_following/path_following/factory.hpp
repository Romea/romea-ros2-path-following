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

#ifndef ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_FACTORY_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_FACTORY_HPP_

// std
#include <memory>
#include <stdexcept>
#include <string>

// romea
#include "romea_path_following/path_following/traits.hpp"

namespace romea::ros2::path_following
{

inline std::string full_name(const std::string & ns, const std::string & name)
{
  if (name.empty()) {
    return ns;
  }
  return ns + "." + name;
}

template<typename LatCtrl, typename LonCtrl, typename SlObs, typename Node>
inline std::unique_ptr<
  core::path_following::PathFollowingWithSlidingObserver<LatCtrl, LonCtrl, SlObs>>
make_path_following(
  std::shared_ptr<Node> node,
  const std::string & lateral_control_name,
  const std::string & longitudinal_control_name,
  const std::string & sliding_observer_name)
{
  return std::make_unique<
    core::path_following::PathFollowingWithSlidingObserver<LatCtrl, LonCtrl, SlObs>>(
    std::make_shared<LatCtrl>(node, full_name("lateral_control", lateral_control_name)),
    std::make_shared<LonCtrl>(node, full_name("longitudinal_control", longitudinal_control_name)),
    std::make_shared<SlObs>(node, full_name("sliding_observer", sliding_observer_name)));
}

template<typename LatCtrl, typename LonCtrl, typename Node>
inline std::unique_ptr<core::path_following::PathFollowingWithoutSlidingObserver<LatCtrl, LonCtrl>>
make_path_following(
  std::shared_ptr<Node> node,
  const std::string & lateral_control_name,
  const std::string & longitudinal_control_name)
{
  return std::make_unique<
    core::path_following::PathFollowingWithoutSlidingObserver<LatCtrl, LonCtrl>>(
    std::make_shared<LatCtrl>(node, full_name("lateral_control", lateral_control_name)),
    std::make_shared<LonCtrl>(node, full_name("longitudinal_control", longitudinal_control_name)));
}

template<typename CommandType>
struct PathFollowingFactory
{
};

template<>
struct PathFollowingFactory<core::OneAxleSteeringCommand>
{
  using Command = core::OneAxleSteeringCommand;
  using Base = PathFollowingTraits<Command>::PathFollowingBase;
  using LonCtrl = PathFollowingTraits<Command>::LongitudinalControl::Classic;
  using LatCtrlClassic = PathFollowingTraits<Command>::LateralControl::Classic;
  using LatCtrlPredictive = PathFollowingTraits<Command>::LateralControl::Predictive;
  using SlObsExtendedCinematic = PathFollowingTraits<Command>::SlidingObserver::ExtendedCinematic;
  using SlObsExtendedLyapunov = PathFollowingTraits<Command>::SlidingObserver::ExtendedLyapunov;

  template<typename Node>
  static std::unique_ptr<Base> make(
    std::shared_ptr<Node> node,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (lateral_control_name == "classic") {
      return make<LatCtrlClassic>(node, lateral_control_name, sliding_observer_name);
    }

    if (lateral_control_name == "predictive") {
      return make<LatCtrlPredictive>(node, lateral_control_name, sliding_observer_name);
    }

    throw std::runtime_error(
      std::string{"Unknown lateral_control '"} + lateral_control_name +
      "'. Available: [classic, predictive]");
  }

  template<typename LatCtrl, typename Node>
  static std::unique_ptr<Base> make(
    std::shared_ptr<Node> node,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (sliding_observer_name == "none") {
      return make_path_following<LatCtrl, LonCtrl>(node, lateral_control_name, "");
    }

    if (sliding_observer_name == "extended_cinematic") {
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedCinematic>(
        node, lateral_control_name, "", sliding_observer_name);
    }

    if (sliding_observer_name == "extended_lyapunov") {
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedLyapunov>(
        node, lateral_control_name, "", sliding_observer_name);
    }

    throw std::runtime_error(
      std::string{"Unknown sliding_observer '"} + sliding_observer_name +
      "'. Available: [none, extended_cinematic, extended_lyapunov]");
  }
};

template<>
struct PathFollowingFactory<core::TwoAxleSteeringCommand>
{
  using Command = core::TwoAxleSteeringCommand;
  using Base = PathFollowingTraits<Command>::PathFollowingBase;
  using LonCtrl = PathFollowingTraits<Command>::LongitudinalControl::Classic;
  using LatCtrlClassic = PathFollowingTraits<Command>::LateralControl::Classic;
  using LatCtrlPredictive = PathFollowingTraits<Command>::LateralControl::Predictive;
  using LatCtrlDecoupled = PathFollowingTraits<Command>::LateralControl::FrontRearDecoupled;
  using SlObsExtendedCinematic = PathFollowingTraits<Command>::SlidingObserver::ExtendedCinematic;
  using SlObsExtendedLyapunov = PathFollowingTraits<Command>::SlidingObserver::ExtendedLyapunov;

  template<typename Node>
  static std::unique_ptr<Base> make(
    std::shared_ptr<Node> node,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (lateral_control_name == "classic") {
      return make<LatCtrlClassic>(node, lateral_control_name, sliding_observer_name);
    }

    if (lateral_control_name == "predictive") {
      return make<LatCtrlPredictive>(node, lateral_control_name, sliding_observer_name);
    }

    if (lateral_control_name == "front_rear_decoupled") {
      return make<LatCtrlDecoupled>(node, lateral_control_name, sliding_observer_name);
    }

    throw std::runtime_error(
      std::string{"Unknown lateral_control '"} + lateral_control_name +
      "'. Available: [classic, predictive, front_rear_decoupled]");
  }

  template<typename LatCtrl, typename Node>
  static std::unique_ptr<Base> make(
    std::shared_ptr<Node> node,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (sliding_observer_name == "none") {
      return make_path_following<LatCtrl, LonCtrl>(node, lateral_control_name, "");
    }

    if (sliding_observer_name == "extended_cinematic") {
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedCinematic>(
        node, lateral_control_name, "", sliding_observer_name);
    }

    if (sliding_observer_name == "extended_lyapunov") {
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedLyapunov>(
        node, lateral_control_name, "", sliding_observer_name);
    }

    throw std::runtime_error(
      std::string{"Unknown sliding_observer '"} + sliding_observer_name +
      "'. Available: [none, extended_cinematic, extended_lyapunov]");
  }
};

template<>
struct PathFollowingFactory<core::SkidSteeringCommand>
{
  using Command = core::SkidSteeringCommand;
  using Base = PathFollowingTraits<Command>::PathFollowingBase;
  using LonCtrl = PathFollowingTraits<Command>::LongitudinalControl::Classic;
  using LatCtrlBackStepping = PathFollowingTraits<Command>::LateralControl::BackStepping;
  using LatCtrlSkidSliding = PathFollowingTraits<Command>::LateralControl::SkidSliding;
  using SOPSBackstepping = PathFollowingTraits<Command>::SlidingObserver::PicardSkidBackstepping;

  template<typename Node>
  static std::unique_ptr<Base> make(
    std::shared_ptr<Node> node,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name,
    bool one_axle_steering_equivalence = false)
  {
    if (!one_axle_steering_equivalence) {
      if (lateral_control_name == "back_stepping") {
        if (sliding_observer_name == "none") {
          return make_path_following<LatCtrlBackStepping, LonCtrl>(node, lateral_control_name, "");
        }
        throw std::runtime_error(
          std::string{"Unknown sliding_observer '"} + sliding_observer_name +
          "'. Available: [none]");
      }

      if (lateral_control_name == "skid_sliding") {
        if (sliding_observer_name == "none") {
          return make_path_following<LatCtrlSkidSliding, LonCtrl>(node, lateral_control_name, "");
        }
        if (sliding_observer_name == "picard_backstepping") {
          return make_path_following<LatCtrlSkidSliding, LonCtrl, SOPSBackstepping>(
            node, lateral_control_name, "", sliding_observer_name);
        }
        throw std::runtime_error(
          std::string{"Unknown sliding_observer '"} + sliding_observer_name +
          "'. Available: [none, picard_backstepping]");
      }
      throw std::runtime_error(
        std::string{"Unknown lateral_control '"} + sliding_observer_name +
        "'. Available: [back_stepping, skid_sliding]");
    }
    return std::make_unique<core::path_following::OneAxleSteeringEquivalence>(
      PathFollowingFactory<core::OneAxleSteeringCommand>::make(
        node, lateral_control_name, sliding_observer_name));
  }
};

}  // namespace romea::ros2::path_following

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_FACTORY_HPP_
