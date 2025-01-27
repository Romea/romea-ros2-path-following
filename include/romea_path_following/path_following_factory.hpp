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
#include "romea_path_following/path_following_lateral_control.hpp"
#include "romea_path_following/path_following_longitudinal_control.hpp"
#include "romea_path_following/path_following_sliding_observer.hpp"
#include "romea_path_following/path_following_traits.hpp"

namespace romea
{
namespace ros2
{

inline std::string full_name(const std::string & ns, const std::string & name)
{
  if (name.empty()) {
    return ns;
  } else {
    return ns + "." + name;
  }
}

template<typename LatCtrl, typename LonCtrl, typename SlObs, typename Node>
inline std::unique_ptr<core::PathFollowingWithSlidingObserver<LatCtrl, LonCtrl, SlObs>>
make_path_following(
  std::shared_ptr<Node> node,
  const std::string & lateral_control_name,
  const std::string & longitudinal_control_name,
  const std::string & sliding_observer_name)
{
  declare_sampling_period(node);
  declare_wheelbase(node);
  declare_inertia(node);
  return std::make_unique<core::PathFollowingWithSlidingObserver<LatCtrl, LonCtrl, SlObs>>(
    make_lateral_control<LatCtrl>(node, full_name("lateral_control", lateral_control_name)),
    make_longitudinal_control<LonCtrl>(
      node, full_name("longitudinal_control", longitudinal_control_name)),
    make_sliding_observer<SlObs>(node, full_name("sliding_observer", sliding_observer_name)));
}

template<typename LatCtrl, typename LonCtrl, typename Node>
inline std::unique_ptr<core::PathFollowingWithoutSlidingObserver<LatCtrl, LonCtrl>>
make_path_following(
  std::shared_ptr<Node> node,
  const std::string & lateral_control_name,
  const std::string & longitudinal_control_name)
{
  declare_sampling_period(node);
  declare_wheelbase(node);
  declare_inertia(node);
  return std::make_unique<core::PathFollowingWithoutSlidingObserver<LatCtrl, LonCtrl>>(
    make_lateral_control<LatCtrl>(node, full_name("lateral_control", lateral_control_name)),
    make_longitudinal_control<LonCtrl>(
      node, full_name("longitudinal_control", longitudinal_control_name)));
}

template<typename CommandType>
struct PathFollowingFactory
{
};

template<>
struct PathFollowingFactory<core::OneAxleSteeringCommand>
{
  using Base = PathFollowingTraits<core::OneAxleSteeringCommand>::PathFollowingBase;
  using LonCtrl = PathFollowingTraits<core::OneAxleSteeringCommand>::LongitudinalControl::Classic;
  using LatCtrlClassic = PathFollowingTraits<core::OneAxleSteeringCommand>::LateralControl::Classic;
  using LatCtrlPredictive =
    PathFollowingTraits<core::OneAxleSteeringCommand>::LateralControl::Predictive;
  using SlObsExtendedCinematic =
    PathFollowingTraits<core::OneAxleSteeringCommand>::SlidingObserver::ExtendedCinematic;
  using SlObsExtendedLyapunov =
    PathFollowingTraits<core::OneAxleSteeringCommand>::SlidingObserver::ExtendedLyapunov;

  template<typename Node>
  static std::unique_ptr<Base> make(
    std::shared_ptr<Node> node,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (lateral_control_name == "classic") {
      return make<LatCtrlClassic>(node, lateral_control_name, sliding_observer_name);
    } else if (lateral_control_name == "predictive") {
      return make<LatCtrlPredictive>(node, lateral_control_name, sliding_observer_name);
    } else {
      // throw
      return nullptr;
    }
  }

  template<typename LatCtrl, typename Node>
  static std::unique_ptr<Base> make(
    std::shared_ptr<Node> node,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (sliding_observer_name == "none") {
      // return nullptr;
      return make_path_following<LatCtrl, LonCtrl>(node, lateral_control_name, "");
    } else if (sliding_observer_name == "extended_cinematic") {
      // return nullptr;
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedCinematic>(
        node, lateral_control_name, "", sliding_observer_name);
    } else if (sliding_observer_name == "extended_lyapunov") {
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedLyapunov>(
        node, lateral_control_name, "", sliding_observer_name);
    } else {
      // throw
      return nullptr;
    }
  }
};

template<>
struct PathFollowingFactory<core::TwoAxleSteeringCommand>
{
  using Base = PathFollowingTraits<core::TwoAxleSteeringCommand>::PathFollowingBase;
  using LonCtrl = PathFollowingTraits<core::TwoAxleSteeringCommand>::LongitudinalControl::Classic;
  using LatCtrlClassic = PathFollowingTraits<core::TwoAxleSteeringCommand>::LateralControl::Classic;
  using LatCtrlPredictive =
    PathFollowingTraits<core::TwoAxleSteeringCommand>::LateralControl::Predictive;
  using SlObsExtendedCinematic =
    PathFollowingTraits<core::TwoAxleSteeringCommand>::SlidingObserver::ExtendedCinematic;
  using SlObsExtendedLyapunov =
    PathFollowingTraits<core::TwoAxleSteeringCommand>::SlidingObserver::ExtendedLyapunov;

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
      std::string{"Unknown lateral_control '"} + sliding_observer_name +
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
      "'. Available: [extended_cinematic, extended_lyapunov]");
  }
};

template<>
struct PathFollowingFactory<core::SkidSteeringCommand>
{
  using Base = PathFollowingTraits<core::SkidSteeringCommand>::PathFollowingBase;
  using LonCtrl = PathFollowingTraits<core::SkidSteeringCommand>::LongitudinalControl::Classic;
  using LatCtrlBackStepping =
    PathFollowingTraits<core::SkidSteeringCommand>::LateralControl::BackStepping;

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
      throw std::runtime_error(
        std::string{"Unknown lateral_control '"} + sliding_observer_name +
        "'. Available: [back_stepping]");
    }
    return std::make_unique<core::OneAxleSteeringEquivalence>(
      PathFollowingFactory<core::OneAxleSteeringCommand>::make(
        node, lateral_control_name, sliding_observer_name));
  }

  // template<typename LatCtrl, typename Node>
  // static std::unique_ptr<Base> make(
  //   std::shared_ptr<Node> node,
  //   const std::string & lateral_control_name,
  //   const std::string & sliding_observer_name)
  // {
  // }
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_FACTORY_HPP_
