/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GZ_SIM_SYSTEMS_ODOMETRY_PUBLISHER_ANGULAR_VELOCITY_HH_
#define GZ_SIM_SYSTEMS_ODOMETRY_PUBLISHER_ANGULAR_VELOCITY_HH_

#include <chrono>
#include <cmath>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

namespace gz::sim::systems::detail
{
/// \brief Calculate angular velocity in the body frame from two world poses.
/// \param[in] _lastPose Pose at the previous update in the world frame.
/// \param[in] _currentPose Pose at the current update in the world frame.
/// \param[in] _dt Elapsed time from the previous to the current update.
/// \return Angular velocity in the current body frame.
inline math::Vector3d CalculateAngularVelocity(
    const math::Pose3d &_lastPose, const math::Pose3d &_currentPose,
    std::chrono::duration<double> _dt)
{
  auto rotationDiff = _currentPose.Rot() * _lastPose.Rot().Inverse();
  rotationDiff.Normalize();
  const math::Vector3d rotationVector(
      rotationDiff.X(), rotationDiff.Y(), rotationDiff.Z());
  const double sinHalfAngle = rotationVector.Length();

  // An identity relative rotation has no defined axis. Its angular velocity
  // is zero, so avoid dividing by the zero-length rotation vector.
  if (sinHalfAngle == 0.0 || _dt.count() <= 0.0)
    return math::Vector3d::Zero;

  // atan2 remains defined when floating-point roundoff puts qw just outside
  // [-1, 1], where acos(qw) would produce NaN.
  const double rotationAngle =
      2.0 * std::atan2(sinHalfAngle, rotationDiff.W());
  const math::Vector3d angularVelocity =
      rotationVector * (rotationAngle / (sinHalfAngle * _dt.count()));

  return _currentPose.Rot().RotateVectorReverse(angularVelocity);
}
}  // namespace gz::sim::systems::detail

#endif  // GZ_SIM_SYSTEMS_ODOMETRY_PUBLISHER_ANGULAR_VELOCITY_HH_
