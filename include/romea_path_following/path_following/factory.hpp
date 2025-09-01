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
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp>
#include <stdexcept>
#include <string>

// romea
#include "romea_path_following/path_following/parameters.hpp"
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
  using Traits = PathFollowingTraits<core::OneAxleSteeringCommand>;
  using Base = Traits::PathFollowingBase;
  using LonCtrlClassic = Traits::LongitudinalControl::Classic;
  using LonCtrlConst = Traits::LongitudinalControl::Constant;
  using LonCtrlCurvTrans = Traits::LongitudinalControl::CurvatureTransition;
  using LatCtrlClassic = Traits::LateralControl::Classic;
  using LatCtrlPredictive = Traits::LateralControl::Predictive;
  using SlObsExtendedCinematic = Traits::SlidingObserver::ExtendedCinematic;
  using SlObsExtendedLyapunov = Traits::SlidingObserver::ExtendedLyapunov;

  template<typename Node>
  static std::unique_ptr<Base> make(
    std::shared_ptr<Node> node,
    const std::string & longitudinal_control,
    const std::string & lateral_control,
    const std::string & sliding_observer)
  {
    if (longitudinal_control == "constant") {
      return make<LonCtrlConst>(node, longitudinal_control, lateral_control, sliding_observer);
    }

    if (longitudinal_control == "classic") {
      return make<LonCtrlClassic>(node, longitudinal_control, lateral_control, sliding_observer);
    }

    if (longitudinal_control == "curvature_transition") {
      return make<LonCtrlCurvTrans>(node, longitudinal_control, lateral_control, sliding_observer);
    }

    throw std::runtime_error(
      std::string{"Unknown longitudinal_control '"} + longitudinal_control +
      "'. Available: [constant, classic, curvature_transition]");
  }

  template<typename LonCtrl, typename Node>
  static std::unique_ptr<Base> make(
    std::shared_ptr<Node> node,
    const std::string & longitudinal_control_name,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (lateral_control_name == "classic") {
      return make<LonCtrl, LatCtrlClassic>(
        node, longitudinal_control_name, lateral_control_name, sliding_observer_name);
    }

    if (lateral_control_name == "predictive") {
      return make<LonCtrl, LatCtrlPredictive>(
        node, longitudinal_control_name, lateral_control_name, sliding_observer_name);
    }

    throw std::runtime_error(
      std::string{"Unknown lateral_control '"} + lateral_control_name +
      "'. Available: [classic, predictive]");
  }

  template<typename LonCtrl, typename LatCtrl, typename Node>
  static std::unique_ptr<Base> make(
    std::shared_ptr<Node> node,
    const std::string & longitudinal_control_name,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (sliding_observer_name == "none") {
      return make_path_following<LatCtrl, LonCtrl>(
        node, lateral_control_name, longitudinal_control_name);
    }

    if (sliding_observer_name == "extended_cinematic") {
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedCinematic>(
        node, lateral_control_name, longitudinal_control_name, sliding_observer_name);
    }

    if (sliding_observer_name == "extended_lyapunov") {
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedLyapunov>(
        node, lateral_control_name, longitudinal_control_name, sliding_observer_name);
    }

    throw std::runtime_error(
      std::string{"Unknown sliding_observer '"} + sliding_observer_name +
      "'. Available: [none, extended_cinematic, extended_lyapunov]");
  }
};

template<>
struct PathFollowingFactory<core::TwoAxleSteeringCommand>
{
  using Traits = PathFollowingTraits<core::TwoAxleSteeringCommand>;
  using Base = Traits::PathFollowingBase;
  using LonCtrlClassic = Traits::LongitudinalControl::Classic;
  using LonCtrlConst = Traits::LongitudinalControl::Constant;
  using LonCtrlCurvTrans = Traits::LongitudinalControl::CurvatureTransition;
  using LatCtrlClassic = Traits::LateralControl::Classic;
  using LatCtrlPredictive = Traits::LateralControl::Predictive;
  using LatCtrlDecoupled = Traits::LateralControl::FrontRearDecoupled;
  using SlObsExtendedCinematic = Traits::SlidingObserver::ExtendedCinematic;
  using SlObsExtendedLyapunov = Traits::SlidingObserver::ExtendedLyapunov;

  template<typename Node>
  static std::unique_ptr<Base> make(
    std::shared_ptr<Node> node,
    const std::string & longitudinal_control,
    const std::string & lateral_control,
    const std::string & sliding_observer)
  {
    if (longitudinal_control == "constant") {
      return make<LonCtrlConst>(node, longitudinal_control, lateral_control, sliding_observer);
    }

    if (longitudinal_control == "classic") {
      return make<LonCtrlClassic>(node, longitudinal_control, lateral_control, sliding_observer);
    }

    if (longitudinal_control == "curvature_transition") {
      return make<LonCtrlCurvTrans>(node, longitudinal_control, lateral_control, sliding_observer);
    }

    throw std::runtime_error(
      std::string{"Unknown longitudinal_control '"} + longitudinal_control +
      "'. Available: [constant, classic, curvature_transition]");
  }

  template<typename LonCtrl, typename Node>
  static std::unique_ptr<Base> make(
    std::shared_ptr<Node> node,
    const std::string & longitudinal_control_name,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (lateral_control_name == "classic") {
      return make<LonCtrl, LatCtrlClassic>(
        node, longitudinal_control_name, lateral_control_name, sliding_observer_name);
    }

    if (lateral_control_name == "predictive") {
      return make<LonCtrl, LatCtrlPredictive>(
        node, longitudinal_control_name, lateral_control_name, sliding_observer_name);
    }

    if (lateral_control_name == "front_rear_decoupled") {
      return make<LonCtrl, LatCtrlDecoupled>(
        node, longitudinal_control_name, lateral_control_name, sliding_observer_name);
    }

    throw std::runtime_error(
      std::string{"Unknown lateral_control '"} + lateral_control_name +
      "'. Available: [classic, predictive, front_rear_decoupled]");
  }

  template<typename LonCtrl, typename LatCtrl, typename Node>
  static std::unique_ptr<Base> make(
    std::shared_ptr<Node> node,
    const std::string & longitudinal_control_name,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (sliding_observer_name == "none") {
      return make_path_following<LatCtrl, LonCtrl>(
        node, lateral_control_name, longitudinal_control_name);
    }

    if (sliding_observer_name == "extended_cinematic") {
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedCinematic>(
        node, lateral_control_name, longitudinal_control_name, sliding_observer_name);
    }

    if (sliding_observer_name == "extended_lyapunov") {
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedLyapunov>(
        node, lateral_control_name, longitudinal_control_name, sliding_observer_name);
    }

    throw std::runtime_error(
      std::string{"Unknown sliding_observer '"} + sliding_observer_name +
      "'. Available: [none, extended_cinematic, extended_lyapunov]");
  }
};

template<>
struct PathFollowingFactory<core::SkidSteeringCommand>
{
  using Traits = PathFollowingTraits<core::SkidSteeringCommand>;
  using Base = Traits::PathFollowingBase;
  using LonCtrlClassic = Traits::LongitudinalControl::Classic;
  using LonCtrlConst = Traits::LongitudinalControl::Constant;
  using LonCtrlCurvTrans = Traits::LongitudinalControl::CurvatureTransition;
  using LatCtrlBackStepping = Traits::LateralControl::BackStepping;
  using LatCtrlSkidSliding = Traits::LateralControl::SkidSliding;
  using LatCtrlGeneric = Traits::LateralControl::DesbosGeneric;
  using LcGenPredHmpc = Traits::LateralControl::DesbosGenericPredictiveHmpc;
  using LcGenPredLmpc = Traits::LateralControl::DesbosGenericPredictiveLmpc;
  using SOPSBackstepping = Traits::SlidingObserver::PicardSkidBackstepping;
  using SOPSLyapunov = Traits::SlidingObserver::PicardSkidLyapunov;

  template<typename Node>
  static std::unique_ptr<Base> make(
    std::shared_ptr<Node> node,
    const std::string & longitudinal_control,
    const std::string & lateral_control,
    const std::string & sliding_observer,
    bool one_axle_steering_equivalence = false)
  {
    if (one_axle_steering_equivalence) {
      return std::make_unique<core::path_following::OneAxleSteeringEquivalence>(
        PathFollowingFactory<core::OneAxleSteeringCommand>::make(
          node, longitudinal_control, lateral_control, sliding_observer),
        try_declare_and_get_wheelbase(node));
    }

    if (longitudinal_control == "constant") {
      return make<LonCtrlConst>(node, longitudinal_control, lateral_control, sliding_observer);
    }

    if (longitudinal_control == "classic") {
      return make<LonCtrlClassic>(node, longitudinal_control, lateral_control, sliding_observer);
    }

    if (longitudinal_control == "curvature_transition") {
      return make<LonCtrlCurvTrans>(node, longitudinal_control, lateral_control, sliding_observer);
    }

    throw std::runtime_error(
      std::string{"Unknown longitudinal_control '"} + longitudinal_control +
      "'. Available: [constant, classic, curvature_transition]");
  }

  template<typename LonCtrl, typename Node>
  static std::unique_ptr<Base> make(
    std::shared_ptr<Node> node,
    const std::string & longitudinal_control_name,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (lateral_control_name == "back_stepping") {
      if (sliding_observer_name == "none") {
        return make_path_following<LatCtrlBackStepping, LonCtrl>(
          node, lateral_control_name, longitudinal_control_name);
      }
      throw std::runtime_error(
        std::string{"Unknown sliding_observer '"} + sliding_observer_name + "'. Available: [none]");
    }

    if (lateral_control_name == "skid_backstepping") {
      return make_sliding<LonCtrl, LatCtrlSkidSliding>(
        node, longitudinal_control_name, lateral_control_name, sliding_observer_name);
    }
    if (lateral_control_name == "desbos_generic") {
      return make_sliding<LonCtrl, LatCtrlGeneric>(
        node, longitudinal_control_name, lateral_control_name, sliding_observer_name);
    }
    if (lateral_control_name == "desbos_generic_predictive_hmpc") {
      return make_sliding<LonCtrl, LcGenPredHmpc>(
        node, longitudinal_control_name, lateral_control_name, sliding_observer_name);
    }
    if (lateral_control_name == "desbos_generic_predictive_lmpc") {
      return make_sliding<LonCtrl, LcGenPredLmpc>(
        node, longitudinal_control_name, lateral_control_name, sliding_observer_name);
    }
    throw std::runtime_error(
      std::string{"Unknown lateral_control '"} + lateral_control_name +
      "'. Available: [back_stepping, skid_backstepping, desbos_generic, "
      "desbos_generic_predictive_hmpc, desbos_generic_predictive_lmpc]");
  }

private:
  template<typename LonCtrl, typename LatCtrl, typename Node>
  static std::unique_ptr<Base> make_sliding(
    std::shared_ptr<Node> node,
    const std::string & longitudinal_control_name,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (sliding_observer_name == "none") {
      return make_path_following<LatCtrl, LonCtrl>(
        node, lateral_control_name, longitudinal_control_name);
    }
    if (sliding_observer_name == "picard_skid_backstepping") {
      return make_path_following<LatCtrl, LonCtrl, SOPSBackstepping>(
        node, lateral_control_name, longitudinal_control_name, sliding_observer_name);
    }
    if (sliding_observer_name == "picard_skid_lyapunov") {
      return make_path_following<LatCtrl, LonCtrl, SOPSLyapunov>(
        node, lateral_control_name, longitudinal_control_name, sliding_observer_name);
    }
    throw std::runtime_error(
      std::string{"Unknown sliding_observer '"} + sliding_observer_name +
      "'. Available: [none, picard_skid_backstepping, picard_skid_lyapunov]");
  }
};

}  // namespace romea::ros2::path_following

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_FACTORY_HPP_
