#ifndef romea_PathMatchingTraits_hpp
#define romea_PathMatchingTraits_hpp

//romea
#include "romea_core_path_following/PathFollowing.hpp"
#include "romea_core_path_following/lateral_control/LateralControlBackStepping.hpp"
#include "romea_core_path_following/lateral_control/LateralControlClassic.hpp"
#include "romea_core_path_following/lateral_control/LateralControlPredictive.hpp"
#include "romea_core_path_following/longitudinal_control/LongitudinalControlClassic.hpp"
#include \
  "romea_core_path_following/sliding_observer/SlidingObserverExtendedCinematicLinearTangent.hpp"
#include "romea_core_path_following/sliding_observer/SlidingObserverExtendedCinematicLyapunov.hpp"


namespace romea
{
namespace ros2
{

template<typename CommandType>
struct PathFollowingTraits
{

};

template<>
struct PathFollowingTraits<core::OneAxleSteeringCommand>
{
  using PathFollowingBase = core::PathFollowingBase<core::OneAxleSteeringCommand>;

  struct LongitudinalControl
  {
    using Classic =
      core::PathFollowingLongitudinalControlClassic<core::OneAxleSteeringCommand>;
  };

  struct LateralControl
  {
    using Classic =
      core::PathFollowingLateralControlClassic<core::OneAxleSteeringCommand>;
    using Predictive =
      core::PathFollowingLateralControlPredictive<core::OneAxleSteeringCommand>;
  };

  struct SlidingObserver
  {
    using ExtendedCinematic =
      core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<core::OneAxleSteeringCommand>;
    using ExtendedLyapunov =
      core::PathFollowingSlidingObserverExtendedCinematicLyapunov<core::OneAxleSteeringCommand>;
  };
};

template<>
struct PathFollowingTraits<core::TwoAxleSteeringCommand>
{
  using PathFollowingBase = core::PathFollowingBase<core::TwoAxleSteeringCommand>;

  struct LongitudinalControl
  {
    using Classic =
      core::PathFollowingLongitudinalControlClassic<core::TwoAxleSteeringCommand>;
  };

  struct LateralControl
  {
    using Classic =
      core::PathFollowingLateralControlClassic<core::TwoAxleSteeringCommand>;
    using Predictive =
      core::PathFollowingLateralControlPredictive<core::TwoAxleSteeringCommand>;
  };

  struct SlidingObserver
  {
    using ExtendedCinematic =
      core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<core::TwoAxleSteeringCommand>;
    using ExtendedLyapunov =
      core::PathFollowingSlidingObserverExtendedCinematicLyapunov<core::TwoAxleSteeringCommand>;
  };
};

template<>
struct PathFollowingTraits<core::SkidSteeringCommand>
{
  using PathFollowingBase = core::PathFollowingBase<core::SkidSteeringCommand>;

  struct LongitudinalControl
  {
    using Classic =
      core::PathFollowingLongitudinalControlClassic<core::SkidSteeringCommand>;
  };

  struct LateralControl
  {
    using BackStepping =
      core::PathFollowingLateralControlBackStepping<core::SkidSteeringCommand>;
  };

  struct SlidingObserver
  {
  };
};

}  // namespace ros2
}  // namespace romea

#endif
