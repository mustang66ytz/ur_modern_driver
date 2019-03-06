#include "ur_modern_driver/ros/trajectory_follower.h"
#include <endian.h>
#include <ros/ros.h>
#include <cmath>

TrajectoryFollower::TrajectoryFollower(URCommander &commander) : running_(false), commander_(commander)
{
}

bool TrajectoryFollower::start()
{
  if (running_)
    return true;  // not sure

  if (!commander_.startServoLoop())
  {
    LOG_ERROR("Failed to start commander");
    return false;
  }

  LOG_DEBUG("Robot successfully connected");
  return (running_ = true);
}

double TrajectoryFollower::interpolate(double t, double T, double p0_pos, double p1_pos, double p0_vel, double p1_vel)
{
  using std::pow;
  double a = p0_pos;
  double b = p0_vel;
  double c = (-3 * a + 3 * p1_pos - 2 * T * b - T * p1_vel) / pow(T, 2);
  double d = (2 * a - 2 * p1_pos + T * b + T * p1_vel) / pow(T, 3);
  return a + b * t + c * pow(t, 2) + d * pow(t, 3);
}

bool TrajectoryFollower::execute(std::array<double, 6> &positions)
{
  return commander_.servoj(positions, true);
}

bool TrajectoryFollower::execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt)
{
  if (!running_)
    return false;

  using namespace std::chrono;
  typedef duration<double> double_seconds;
  typedef high_resolution_clock Clock;
  typedef Clock::time_point Time;

  auto &last = trajectory[trajectory.size() - 1];
  auto &prev = trajectory[0];

  Time t0 = Clock::now();
  Time latest = t0;

  std::array<double, 6> positions;

  for (auto const &point : trajectory)
  {
    // skip t0
    if (&point == &prev)
      continue;

    if (interrupt)
      break;

    auto duration = point.time_from_start - prev.time_from_start;
    double d_s = duration_cast<double_seconds>(duration).count();

    // interpolation loop
    while (!interrupt)
    {
      latest = Clock::now();
      auto elapsed = latest - t0;

      if (point.time_from_start <= elapsed)
        break;

      if (last.time_from_start <= elapsed)
        return true;

      double elapsed_s = duration_cast<double_seconds>(elapsed - prev.time_from_start).count();
      for (size_t j = 0; j < positions.size(); j++)
      {
        positions[j] =
            interpolate(elapsed_s, d_s, prev.positions[j], point.positions[j], prev.velocities[j], point.velocities[j]);
      }

      if (!commander_.servoj(positions, true))
        return false;

      std::this_thread::sleep_for(std::chrono::milliseconds((int)((commander_.servojTime() * 1000) / 4.)));
    }

    prev = point;
  }

  // In theory it's possible the last position won't be sent by
  // the interpolation loop above but rather some position between
  // t[N-1] and t[N] where N is the number of trajectory points.
  // To make sure this does not happen the last position is sent
  return commander_.servoj(last.positions, true);
}

void TrajectoryFollower::stop()
{
  if (!running_)
    return;

  // std::array<double, 6> empty;
  // execute(empty, false);
  commander_.stopServoLoop();

  running_ = false;
}
