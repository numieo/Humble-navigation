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

#include "dwb_critics/goal_dist.hpp"
#include <vector>
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/path_ops.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace dwb_critics
{
// 目标距离批判器（critic），用于评估机器人距离目标的路径规划的质量。
// 这个批判器的目的是帮助机器人产生能够更好地接近目标的路径
bool GoalDistCritic::prepare(
  const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  // 用于在准备新的路径规划前清空之前的状态。它会将内部维护的距离信息以及待处理的队列进行重置
  reset();
  // 根据全局路径规划获取目标点在代价地图上的坐标（即栅格坐标）。如果找不到目标点在代价地图上的坐标，函数将返回false，否则返回true
  unsigned int local_goal_x, local_goal_y;
  if (!getLastPoseOnCostmap(global_plan, local_goal_x, local_goal_y)) {
    return false;
  }

  // Enqueue just the last pose
  // 通过目标点在代价地图上的坐标，计算其在代价地图中的索引
  int index = costmap_->getIndex(local_goal_x, local_goal_y);
  // 设置目标点在代价地图中的距离值为0，表示目标点本身的距离为0
  cell_values_[index] = 0.0;
  // 将目标点加入到待处理的队列中，以便进行后续的距离传播计算
  queue_->enqueueCell(local_goal_x, local_goal_y);
  // 这是一个距离传播函数，它使用曼哈顿距离传播算法，从目标点开始向周围的栅格进行距离传播。根据代价地图中的障碍物情况，计算每个栅格到目标点的距离
  propogateManhattanDistances();
  // 表示准备过程成功完成，可以进行后续的评估
  return true;
}
// 将全局路径规划进行调整，使其分辨率与代价地图一致。这是为了确保路径上的点能够正确地对应到代价地图上的栅格
bool GoalDistCritic::getLastPoseOnCostmap(
  const nav_2d_msgs::msg::Path2D & global_plan,
  unsigned int & x, unsigned int & y)
{
  nav_2d_msgs::msg::Path2D adjusted_global_plan = nav_2d_utils::adjustPlanResolution(
    global_plan,
    costmap_->getResolution());
  // 用于表示是否已经开始遍历路径
  bool started_path = false;

  // skip global path points until we reach the border of the local map
  for (unsigned int i = 0; i < adjusted_global_plan.poses.size(); ++i) 
  {
    // 获取调整后的全局路径规划中的当前点的x和y坐标
    double g_x = adjusted_global_plan.poses[i].x;
    double g_y = adjusted_global_plan.poses[i].y;
    // 用于存储当前点在代价地图上的栅格坐标
    unsigned int map_x, map_y;
    // 首先，将当前点的世界坐标转换为代价地图上的栅格坐标。然后，检查该栅格单元是否有效
    // （即，不在地图的边界之外），以及其代价是否不是NO_INFORMATION（即，是否有有效的代价信息）
    if (costmap_->worldToMap(
        g_x, g_y, map_x,
        map_y) && costmap_->getCost(map_x, map_y) != nav2_costmap_2d::NO_INFORMATION)
    {
      // Still on the costmap. Continue.
      // 如果满足上述条件，表示当前点位于代价地图上，并且代价是有效的。将x和y设置为当前点的栅格坐标，
      // 并将started_path设置为true，表示已经开始遍历路径上的点
      x = map_x;
      y = map_y;
      started_path = true;
    } else if (started_path) {
      // 如果不满足条件，说明当前点不在代价地图上或者其代价无效。在这种情况下，检查started_path的状态。
      // 如果之前已经找到代价地图上的点，那么说明我们之前在代价地图上，但现在已经离开代价地图，因此返回之前保存的最后一个有效点的坐标
      // Off the costmap after being on the costmap. Return the last saved indices.
      return true;
    }
    // else, we have not yet found a point on the costmap, so we just continue
    // 如果之前没有找到代价地图上的点，就继续遍历下一个点，因为当前点不在代价地图上
  }

  // 通过检查started_path的状态，如果之前已经找到了代价地图上的点，那么函数将返回true，
  // 表示成功找到路径上最后一个位于代价地图上的点的栅格坐标。如果整个路径上的点都不在代价地图上，
  // 函数将输出一个错误消息并返回false
  if (started_path) {
    return true;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "GoalDistCritic"), "None of the points of the global plan were in the local costmap.");
    return false;
  }
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::GoalDistCritic, dwb_core::TrajectoryCritic)
