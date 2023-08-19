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

#include "dwb_critics/goal_align.hpp"
#include <vector>
#include <string>
#include "dwb_critics/alignment_util.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/parameters.hpp"

namespace dwb_critics
{

void GoalAlignCritic::onInit()
{
  GoalDistCritic::onInit();
  stop_on_failure_ = false;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  forward_point_distance_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".forward_point_distance", 0.325);
}

/**
 * @brief 为评估准备数据,目标是计算出机器人在全局路径上的某一点，使得当前位置指向该点的直线方向相一致，
 * 而不是路径的最后一点。这是为了避免在机器人需要在最后一点进行180度转弯时出现不稳定性
 * 
 * @param pose 当前机器人位姿
 * @param vel 速度
 * @param goal 目标位姿
 * @param global_plan 全局路径
 * @return true 
 * @return false 
 */
bool GoalAlignCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  // we want the robot nose to be drawn to its final position
  // (before robot turns towards goal orientation), not the end of the
  // path for the robot center. Choosing the final position after
  // turning towards goal orientation causes instability when the
  // robot needs to make a 180 degree turn at the end

  // 计算当前机器人位姿到目标位姿的方向角 angle_to_goal
  double angle_to_goal = atan2(goal.y - pose.y, goal.x - pose.x);
  // 将 target_poses 的最后一个点沿着 angle_to_goal 方向延伸一定距离 forward_point_distance_，从而得到机器人的预期最终位置
  nav_2d_msgs::msg::Path2D target_poses = global_plan;
  target_poses.poses.back().x += forward_point_distance_ * cos(angle_to_goal);
  target_poses.poses.back().y += forward_point_distance_ * sin(angle_to_goal);
  // 调用基类 GoalDistCritic 的 prepare 函数，以计算机器人在目标对齐后的位置上的路径规划评分
  return GoalDistCritic::prepare(pose, vel, goal, target_poses);
}
// 评估给定位姿的得分
double GoalAlignCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  // 评估机器人在预期的目标对齐位置上的路径规划得分
  return GoalDistCritic::scorePose(getForwardPose(pose, forward_point_distance_));
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::GoalAlignCritic, dwb_core::TrajectoryCritic)
