/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
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

#include <dwb_core/trajectory_utils.hpp>

#include <cmath>

#include "rclcpp/duration.hpp"

#include "dwb_core/exceptions.hpp"

namespace dwb_core
{
// getClosestPose 函数用于从给定的轨迹中找到与指定时间偏移最接近的位姿（Pose2D）

const geometry_msgs::msg::Pose2D & getClosestPose(
  const dwb_msgs::msg::Trajectory2D & trajectory, // 输入的轨迹
  const double time_offset)   // 目标时间偏移量，以秒为单位
{
  rclcpp::Duration goal_time = rclcpp::Duration::from_seconds(time_offset); // 将 time_offset 转换为 rclcpp::Duration，以便将其与轨迹中的时间偏移进行比较
  const unsigned int num_poses = trajectory.poses.size();
  if (num_poses == 0) {       // 检查轨迹是否为空，如果为空，则抛出异常
    throw nav2_core::PlannerException("Cannot call getClosestPose on empty trajectory.");
  }
  unsigned int closest_index = num_poses; // 存储最接近位姿的索引
  double closest_diff = 0.0;              // 时间偏移的差值
  // 遍历轨迹中的每个位姿，计算当前位姿的时间偏移与目标时间偏移之间的差值
  for (unsigned int i = 0; i < num_poses; ++i) {
    double diff = std::fabs((rclcpp::Duration(trajectory.time_offsets[i]) - goal_time).seconds());
    // 如果当前的差值更接近目标差值，则更新 closest_index 和 closest_diff
    if (closest_index == num_poses || diff < closest_diff) {
      closest_index = i;
      closest_diff = diff;
    }
    // 如果目标时间偏移小于当前位姿的时间偏移，意味着已经找到了最接近的位姿，因此退出循环
    if (goal_time < rclcpp::Duration(trajectory.time_offsets[i])) {
      break;
    }
  }
  // 返回位姿数组中最接近的位姿
  return trajectory.poses[closest_index];
}

geometry_msgs::msg::Pose2D projectPose(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double time_offset)
{
  // 将 time_offset 转换为 rclcpp::Duration，以便将其与轨迹中的时间偏移进行比较
  rclcpp::Duration goal_time = rclcpp::Duration::from_seconds(time_offset);
  const unsigned int num_poses = trajectory.poses.size();
  // 检查轨迹是否为空，如果为空，则抛出异常
  if (num_poses == 0) {
    throw nav2_core::PlannerException("Cannot call projectPose on empty trajectory.");
  }
  // 检查目标时间偏移是否小于或等于轨迹的第一个时间偏移，如果是，则返回轨迹的第一个位姿
  if (goal_time <= (trajectory.time_offsets[0])) {
    return trajectory.poses[0];
  } 
  // 检查目标时间偏移是否大于或等于轨迹的最后一个时间偏移，如果是，则返回轨迹的最后一个位姿
  else if (goal_time >= rclcpp::Duration(trajectory.time_offsets[num_poses - 1])) {
    return trajectory.poses[num_poses - 1];
  }
  // 遍历轨迹中的每个位姿，检查目标时间偏移是否在当前时间偏移和下一个时间偏移之间
  for (unsigned int i = 0; i < num_poses - 1; ++i) {
    // 如果是，计算当前时间偏移和下一个时间偏移之间的时间差 time_diff
    if (goal_time >= rclcpp::Duration(trajectory.time_offsets[i]) &&
      goal_time < rclcpp::Duration(trajectory.time_offsets[i + 1]))
    {
      double time_diff =
        (rclcpp::Duration(trajectory.time_offsets[i + 1]) -
        rclcpp::Duration(trajectory.time_offsets[i])).seconds();
      // 计算目标时间偏移相对于当前时间偏移的比例 ratio，以及相对于下一个时间偏移的反比例 inv_ratio
      double ratio = (goal_time - rclcpp::Duration(trajectory.time_offsets[i])).seconds() /
        time_diff;
      double inv_ratio = 1.0 - ratio;
      const geometry_msgs::msg::Pose2D & pose_a = trajectory.poses[i];
      const geometry_msgs::msg::Pose2D & pose_b = trajectory.poses[i + 1];
      // 插值计算位姿的 x、y 和角度 theta 分量，将插值结果存储在 projected 变量中
      geometry_msgs::msg::Pose2D projected;
      projected.x = pose_a.x * inv_ratio + pose_b.x * ratio;
      projected.y = pose_a.y * inv_ratio + pose_b.y * ratio;
      projected.theta = pose_a.theta * inv_ratio + pose_b.theta * ratio;
      return projected;   // 返回计算出的插值位姿 
    }
  }

  // Should not reach this point
  // 如果目标时间偏移不在任何两个连续时间偏移之间，代码会在最后一个位姿上返回
  return trajectory.poses[num_poses - 1];
}


}  // namespace dwb_core
