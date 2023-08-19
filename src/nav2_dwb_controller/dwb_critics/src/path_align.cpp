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

#include "dwb_critics/path_align.hpp"
#include <vector>
#include <string>
#include "dwb_critics/alignment_util.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/parameters.hpp"

namespace dwb_critics
{

void PathAlignCritic::onInit()
{
  PathDistCritic::onInit();
  // 标识在路径对齐失败时是否需要停止导航。在这里，将其设置为 false，表示不会停止导航
  stop_on_failure_ = false;

  // 尝试通过 node_ 的 lock() 方法获取一个弱引用的 node 指针。node_ 通常是一个指向节点的弱指针
  auto node = node_.lock();
  // 如果获取失败，说明节点对象不可用，会抛出一个运行时错误，表示无法锁定节点
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  // 从节点参数中获取路径对齐评价器的前向点距离参数
  // 如果获取参数失败，则默认设置为 0.325
  forward_point_distance_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".forward_point_distance", 0.325);
}

// 用于准备评价器的数据，并判断是否需要进行路径对齐

bool PathAlignCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  // 计算机器人当前位置与目标位置之间的横向和纵向差值
  double dx = pose.x - goal.x;
  double dy = pose.y - goal.y;
  // 计算机器人当前位置与目标位置之间的平方距离
  double sq_dist = dx * dx + dy * dy;
  // 判断语句用于检查机器人是否距离目标位置足够远，以决定是否进行路径对齐
  if (sq_dist > forward_point_distance_ * forward_point_distance_) {
    zero_scale_ = false;
  }
  // 如果机器人距离目标位置的平方距离小于等于前向点距离的平方，表示机器人已经接近目标位置
  else {
    // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
    // 一旦接近目标，尝试让机器人的头部保持靠近任何物体会破坏行为，因此在这种情况下将 zero_scale_ 设置为 true，表示不进行路径对齐
    zero_scale_ = true;
    return true;
  }
  // 传递机器人的当前姿态、速度、目标姿态和全局路径，以进行进一步的准备工作。基类的 prepare() 方法可能会进行路径距离评估等操作
  return PathDistCritic::prepare(pose, vel, goal, global_plan);
}

// 返回评价器的缩放因子

double PathAlignCritic::getScale() const
{
  // 如果 zero_scale_ 为 true，表示不需要进行路径对齐，因此返回缩放因子 0.0，即没有评价效果
  if (zero_scale_) {
    return 0.0;
  } 
  // 计算得到的值的一半返回作为评价器的缩放因子
  else {
    return costmap_->getResolution() * 0.5 * scale_;
  }
}

// 用于对给定的姿态进行评分

double PathAlignCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  // 获取当前姿态向前移动一定距离的新姿态，然后调用基类 PathDistCritic 的 scorePose() 方法对这个新姿态进行评分。这个评分可能涉及路径距离等因素
  return PathDistCritic::scorePose(getForwardPose(pose, forward_point_distance_));
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathAlignCritic, dwb_core::TrajectoryCritic)
