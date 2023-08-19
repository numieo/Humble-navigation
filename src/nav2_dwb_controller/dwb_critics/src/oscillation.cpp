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

#include "dwb_critics/oscillation.hpp"
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include "nav_2d_utils/parameters.hpp"
#include "nav2_util/node_utils.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::OscillationCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

// 初始化命令趋势的初始状态

OscillationCritic::CommandTrend::CommandTrend()
{
  reset();
}

//  重置命令趋势的状态，将其恢复到初始状态

void OscillationCritic::CommandTrend::reset()
{
  sign_ = Sign::ZERO;
  positive_only_ = false;
  negative_only_ = false;
}

// 更新命令趋势的状态，并检测是否发生趋势变化

bool OscillationCritic::CommandTrend::update(double velocity)
{
  bool flag_set = false;
  // 如果 velocity 小于 0，表示机器人向后运动，如果之前的趋势是正向（向前），
  // 则将 negative_only_ 设置为 true，表示只有负向趋势，然后将 sign_ 设置为 Sign::NEGATIVE
  if (velocity < 0.0) {
    if (sign_ == Sign::POSITIVE) {
      negative_only_ = true;
      flag_set = true;
    }
    sign_ = Sign::NEGATIVE;
  } 
  // 如果 velocity 大于 0，表示机器人向前运动，如果之前的趋势是负向（向后），
  // 则将 positive_only_ 设置为 true，表示只有正向趋势，然后将 sign_ 设置为 Sign::POSITIVE
  else if (velocity > 0.0) {
    if (sign_ == Sign::NEGATIVE) {
      positive_only_ = true;
      flag_set = true;
    }
    sign_ = Sign::POSITIVE;
  }
  return flag_set;
}

// 判断机器人是否处于震荡状态。如果 positive_only_ 为 true（表示只有正向趋势），并且当前速度
// 小于 0，或者 negative_only_ 为 true（表示只有负向趋势），并且当前速度大于 0，则返回 true，表示机器人处于震荡状态

bool OscillationCritic::CommandTrend::isOscillating(double velocity)
{
  return (positive_only_ && velocity < 0.0) || (negative_only_ && velocity > 0.0);
}

// 检测命令趋势是否发生了正负号的变化。如果 positive_only_ 或 negative_only_ 中有任何一个
// 为 true，则表示命令趋势在正负号上发生了变化，函数返回 true，否则返回 false

bool OscillationCritic::CommandTrend::hasSignFlipped()
{
  return positive_only_ || negative_only_;
}

void OscillationCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  clock_ = node->get_clock();

  oscillation_reset_dist_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".oscillation_reset_dist", 0.05);
  oscillation_reset_dist_sq_ = oscillation_reset_dist_ * oscillation_reset_dist_;
  oscillation_reset_angle_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".oscillation_reset_angle", 0.2);
  oscillation_reset_time_ = rclcpp::Duration::from_seconds(
    nav_2d_utils::searchAndGetParam(
      node,
      dwb_plugin_name_ + "." + name_ + ".oscillation_reset_time", -1.0));

  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".x_only_threshold", rclcpp::ParameterValue(0.05));

  /**
   * Historical Parameter Loading
   * If x_only_threshold is set, use that.
   * If min_speed_xy is set in the namespace (as it is often used for trajectory generation), use that.
   * If min_trans_vel is set in the namespace, as it used to be used for trajectory generation, complain then use that.
   * Otherwise, set x_only_threshold_ to 0.05
   */
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".x_only_threshold", x_only_threshold_);
  // TODO(crdelsey): How to handle searchParam?
  // std::string resolved_name;
  // if (node->hasParam("x_only_threshold"))
  // {
  //   node->param("x_only_threshold", x_only_threshold_);
  // }
  // else if (node->searchParam("min_speed_xy", resolved_name))
  // {
  //   node->param(resolved_name, x_only_threshold_);
  // }
  // else if (node->searchParam("min_trans_vel", resolved_name))
  // {
  //   ROS_WARN_NAMED("OscillationCritic",
  //     "Parameter min_trans_vel is deprecated. "
  //     "Please use the name min_speed_xy or x_only_threshold instead.");
  //   node->param(resolved_name, x_only_threshold_);
  // }
  // else
  // {
  //   x_only_threshold_ = 0.05;
  // }

  reset();
}

// 在每次规划新轨迹之前调用，用于准备震荡检测。它将传递的当前
// 机器人位姿 pose 存储到类成员变量 pose_ 中，并返回 true 表示准备完成

bool OscillationCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D &)
{
  pose_ = pose;
  return true;
}

void OscillationCritic::debrief(const nav_2d_msgs::msg::Twist2D & cmd_vel)
{
  // 设置命令趋势标志，以检测机器人是否震荡
  if (setOscillationFlags(cmd_vel)) {
    prev_stationary_pose_ = pose_;       // 记录之前的静止位姿
    prev_reset_time_ = clock_->now();    // 记录重置时间
  }

  // if we've got restrictions... check if we can reset any oscillation flags
  // 如果任何趋势发生了变化
  if (x_trend_.hasSignFlipped() || y_trend_.hasSignFlipped() || theta_trend_.hasSignFlipped()) {
    // Reset flags if enough time or distance has passed
    // 如果可以重置震荡标志（足够时间或距离）
    if (resetAvailable()) {
      reset();  // 执行重置操作
    }
  }
}

// 当机器人在导航过程中遇到障碍物或其他情况时，可能会出现震荡现象，即来回摆动而没有实际进展。
// 这段代码是为了检测这种情况，并在需要时执行重置操作，以确保机器人能够继续正常导航

bool OscillationCritic::resetAvailable()
{
  // 判断是否达到了震荡重置的距离阈值
  if (oscillation_reset_dist_ >= 0.0) {   // 检查是否达到了震荡重置的距离阈值
    // 如果 oscillation_reset_dist_ 大于等于零，则需要计算当前机器人位置与上一次静止位置的距离
    double x_diff = pose_.x - prev_stationary_pose_.x;
    double y_diff = pose_.y - prev_stationary_pose_.y;
    // 并计算距离的平方 sq_dist
    double sq_dist = x_diff * x_diff + y_diff * y_diff;
    // 如果 sq_dist 大于预先设定的震荡重置距离的平方 oscillation_reset_dist_sq_
    if (sq_dist > oscillation_reset_dist_sq_) {
      // 表示需要进行重置
      return true;
    }
  }
  // 判断是否达到了震荡重置的角度阈值
  if (oscillation_reset_angle_ >= 0.0) {
    // 如果 oscillation_reset_angle_ 大于等于零，则计算当前机器人的角度变化 th_diff
    // 计算当前机器人的角度变化 th_diff，即当前角度与上一次静止位置的角度差
    double th_diff = pose_.theta - prev_stationary_pose_.theta;
    if (fabs(th_diff) > oscillation_reset_angle_) {
      // 如果 th_diff 的绝对值大于预先设定的震荡重置角度阈值，就返回 true, 表示需要进行重置
      return true;
    }
  }
  // 判断是否达到了震荡重置的时间阈值
  // 如果 oscillation_reset_time_ 大于等于零，则计算当前时间与上一次重置时间的时间差 t_diff
  if (oscillation_reset_time_ >= rclcpp::Duration::from_seconds(0.0)) {
    auto t_diff = (clock_->now() - prev_reset_time_);
    // 如果 t_diff 大于预先设定的震荡重置时间阈值，就返回 true，表示需要进行重置
    if (t_diff > oscillation_reset_time_) {
      return true;
    }
  }
  // 如果以上三个条件都不满足，则表示没有达到需要重置的条件，函数返回 false，不需要进行重置
  return false;
}

void OscillationCritic::reset()
{
  x_trend_.reset();
  y_trend_.reset();
  theta_trend_.reset();
}

// 设置与震荡相关的标志位，以帮助检测机器人在导航过程中是否出现了震荡情况

bool OscillationCritic::setOscillationFlags(const nav_2d_msgs::msg::Twist2D & cmd_vel)
{
  bool flag_set = false;
  // set oscillation flags for moving forward and backward
  // 设置前进和后退震荡标志位
  // 调用了 x_trend_ 对象的 update() 方法，并传递了机器人的线性速度 cmd_vel.x 作为参数。
  // x_trend_ 用于跟踪机器人前进或后退的趋势。如果机器人的线性速度发生变化，这个函数可能会设置相应的标志位
  flag_set |= x_trend_.update(cmd_vel.x);

  // we'll only set flags for strafing and rotating when we're not moving forward at all
  // 只有当不以全速前进时，才会设置侧向移动和旋转的标志位
  // 检查是否满足设置侧向移动和旋转标志位的条件。x_only_threshold_ 是一个阈值，用于判断机器人
  // 是否在以全速前进。如果 x_only_threshold_ 小于零或者机器人的线性速度的绝对值小于等于
  //  x_only_threshold_，则进入条件块内
  if (x_only_threshold_ < 0.0 || fabs(cmd_vel.x) <= x_only_threshold_) {
    flag_set |= y_trend_.update(cmd_vel.y);         // 跟踪机器人的侧向移动趋势
    flag_set |= theta_trend_.update(cmd_vel.theta); // 跟踪机器人的旋转趋势
  }
  return flag_set;    // 表示是否设置了震荡相关的标志位。如果有任何一个趋势发生了变化，flag_set 将为 true，表示可能出现了震荡情况
}

double OscillationCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  // 调用了 x_trend_ 对象的 isOscillating() 方法，并传递了轨迹中的线性速度 traj.velocity.x 
  // 作为参数。isOscillating() 方法用于判断给定的速度是否表示机器人正在发生震荡
  if (x_trend_.isOscillating(traj.velocity.x) ||
    //  判断轨迹中的侧向速度traj.velocity.y 是否表示机器人正在发生侧向震荡
    y_trend_.isOscillating(traj.velocity.y) ||
    // 判断轨迹中的角速度 traj.velocity.theta 是否表示机器人正在发生旋转震荡
    theta_trend_.isOscillating(traj.velocity.theta))
  {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory is oscillating.");
  }
  // 如果没有检测到轨迹出现震荡情况，函数会返回评分值 0.0，表示这条轨迹是正常的、没有震荡的
  return 0.0;
}

}  // namespace dwb_critics
