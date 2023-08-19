/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "dwb_critics/rotate_to_goal.hpp"
#include <string>
#include <vector>
#include "nav_2d_utils/parameters.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "angles/angles.h"

PLUGINLIB_EXPORT_CLASS(dwb_critics::RotateToGoalCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

inline double hypot_sq(double dx, double dy)
{
  return dx * dx + dy * dy;
}

void RotateToGoalCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  xy_goal_tolerance_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".xy_goal_tolerance", 0.25);
  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
  double stopped_xy_velocity = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".trans_stopped_velocity", 0.25);
  stopped_xy_velocity_sq_ = stopped_xy_velocity * stopped_xy_velocity;
  slowing_factor_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".slowing_factor", 5.0);
  lookahead_time_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".lookahead_time", -1.0);
  reset();
}

void RotateToGoalCritic::reset()
{
  in_window_ = false;
  rotating_ = false;
}

bool RotateToGoalCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D &)
{
  // 计算机器人当前位置与目标位置的平方距离
  double dxy_sq = hypot_sq(pose.x - goal.x, pose.y - goal.y);
  // 表示机器人与目标点之间的距离要小于等于这个阈值才认为在窗口内。如果 dxy_sq 小于等于 xy_goal_tolerance_sq_，表示机器人在窗口内
  in_window_ = in_window_ || dxy_sq <= xy_goal_tolerance_sq_;
  // 计算当前机器人的平面速度的平方
  // 计算当前机器人的平面速度的平方。vel.x 和 vel.y 是机器人的 x 和 y 方向的速度
  current_xy_speed_sq_ = hypot_sq(vel.x, vel.y);
  // 判断机器人是否正在旋转。如果机器人在旋转窗口内且平面速度的平方小于等于
  //  stopped_xy_velocity_sq_，表示机器人的平面速度很小，认为机器人正在旋转，将 rotating_ 设置为 true
  rotating_ = rotating_ || (in_window_ && current_xy_speed_sq_ <= stopped_xy_velocity_sq_);
  // 将目标点的角度存储在 goal_yaw_ 变量中
  goal_yaw_ = goal.theta;
  // 表示准备工作成功
  return true;
}

double RotateToGoalCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  // If we're not sufficiently close to the goal, we don't care what the twist is
  // 如果机器人与目标不足够接近（不在旋转窗口内），直接返回得分 0.0，表示不关心速度，只关心机器人是否在旋转窗口内
  if (!in_window_) {
    return 0.0;
  } 
  // 如果机器人在旋转窗口内且不在旋转状态
  else if (!rotating_) {
    // 计算轨迹中的线性速度的平方
    double speed_sq = hypot_sq(traj.velocity.x, traj.velocity.y);
    // 如果轨迹中的速度平方大于等于当前平面速度的平方，表示机器人没有减速，将抛出异常
    if (speed_sq >= current_xy_speed_sq_) {
      throw dwb_core::IllegalTrajectoryException(name_, "Not slowing down near goal.");
    }
    // 返回速度的平方乘以减速因子，加上旋转的得分
    return speed_sq * slowing_factor_ + scoreRotation(traj);
  }
  // 如果在旋转状态
  // If we're sufficiently close to the goal, any transforming velocity is invalid
  // 如果轨迹中存在非零的线性速度，表示存在无效的变换速度，将抛出异常
  if (fabs(traj.velocity.x) > 0 || fabs(traj.velocity.y) > 0) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Nonrotation command near goal.");
  }
  // 返回旋转的得分
  return scoreRotation(traj);
}

double RotateToGoalCritic::scoreRotation(const dwb_msgs::msg::Trajectory2D & traj)
{
  // 检查轨迹中是否存在点。如果轨迹为空，即没有点，就会抛出异常，表示无法计算旋转得分
  if (traj.poses.empty()) {
    throw dwb_core::IllegalTrajectoryException(name_, "Empty trajectory.");
  }
  // 声明一个名为 end_yaw 的变量，用于存储机器人最终的朝向
  double end_yaw;
  // 如果设置了前瞻时间，执行这个条件分支
  if (lookahead_time_ >= 0.0) {
    // 计算轨迹上前瞻时间处的姿态
    geometry_msgs::msg::Pose2D eval_pose = dwb_core::projectPose(traj, lookahead_time_);
    // 将计算得到的姿态中的朝向存储在 end_yaw 变量中
    end_yaw = eval_pose.theta;
  } 
  // 如果没有设置前瞻时间，直接使用轨迹中最后一个点的朝向作为机器人的最终朝向
  else {
    end_yaw = traj.poses.back().theta;
  }
  // 计算机器人的最终朝向 end_yaw 与目标朝向 goal_yaw_ 之间的角度差，
  // 并返回这个角度差的绝对值。这个角度差越小，表示机器人的朝向与目标朝向越接近，得分越高。
  return fabs(angles::shortest_angular_distance(end_yaw, goal_yaw_));
}

}  // namespace dwb_critics
