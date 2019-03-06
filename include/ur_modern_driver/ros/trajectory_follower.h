/*
 * Copyright 2017, 2018 Jarek Potiuk (low bandwidth trajectory follower)
 *
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
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

#pragma once

#include <inttypes.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <string>
#include <thread>
#include <vector>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ros/action_trajectory_follower_interface.h"
#include "ur_modern_driver/ur/commander.h"

class TrajectoryFollower : public ActionTrajectoryFollowerInterface
{
private:
  std::atomic<bool> running_;
  std::array<double, 6> last_positions_;
  URCommander &commander_;

  bool execute(std::array<double, 6> &positions, bool keep_alive);
  double interpolate(double t, double T, double p0_pos, double p1_pos, double p0_vel, double p1_vel);

public:
  TrajectoryFollower(URCommander &commander);

  bool start();
  bool execute(std::array<double, 6> &positions);
  bool execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt);
  void stop();

  virtual ~TrajectoryFollower(){};
};
