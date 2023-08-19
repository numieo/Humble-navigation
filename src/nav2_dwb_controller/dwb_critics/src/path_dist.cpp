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

#include "dwb_critics/path_dist.hpp"
#include <vector>
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/path_ops.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace dwb_critics
{
// 准备评价器的数据
bool PathDistCritic::prepare(
  const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  reset();  //重置评价器的状态，准备接收新的数据进行评估
  bool started_path = false;
  // 对全局路径进行分辨率调整
  nav_2d_msgs::msg::Path2D adjusted_global_plan =
    nav_2d_utils::adjustPlanResolution(global_plan, costmap_->getResolution());
  //检查调整分辨率后的路径长度是否与原始全局路径长度相同
  if (adjusted_global_plan.poses.size() != global_plan.poses.size()) {
    // 如果长度不相同，表示路径经过了调整，此时会输出一条调试信息，记录调整的点数
    RCLCPP_DEBUG(
      rclcpp::get_logger(
        "PathDistCritic"), "Adjusted global plan resolution, added %zu points",
      adjusted_global_plan.poses.size() - global_plan.poses.size());
  }

  unsigned int i;
  // put global path points into local map until we reach the border of the local map
  // 将全局路径点放入局部地图，直到达到局部地图的边界
  // 从全局路径中的每个点开始，将这些点映射到局部代价地图上，直到遇到局部地图的边界或遇到无效的点
  for (i = 0; i < adjusted_global_plan.poses.size(); ++i) {
    // 从经过分辨率调整后的路径中获取第 i 个点的 x 和 y 坐标
    double g_x = adjusted_global_plan.poses[i].x;
    double g_y = adjusted_global_plan.poses[i].y;
    // 存储将全局坐标转换为地图坐标后的结果
    unsigned int map_x, map_y;
    // 将全局坐标转换为地图坐标
    if (costmap_->worldToMap(
        g_x, g_y, map_x,
        map_y) && costmap_->getCost(map_x, map_y) != nav2_costmap_2d::NO_INFORMATION)
    {
      // 获取地图上的索引并将对应的格子值设置为0
      int index = costmap_->getIndex(map_x, map_y);
      // 表示这个点在局部地图上是可到达的
      cell_values_[index] = 0.0;
      // 将格子添加到评分队列中
      queue_->enqueueCell(map_x, map_y);
      // 将这个格子添加到评分队列中
      started_path = true;
    } 
    // 如果点无效，并且之前已经开始处理路径：如果遇到了无效的点，并且之前已经开始处理路径中的有效点，就跳出循环，不再处理后续无效的点
    else if (started_path) {
      break;
    }
  }
  // 如果在之前的循环中没有开始处理路径中的有效点
  if (!started_path) {
    // 指出在全局计划的前 i 个点中，没有任何点在局部代价地图中是可达且自由的。
    // 路径规划可能无法在代价地图的有效区域内找到可行路径
    RCLCPP_ERROR(
      rclcpp::get_logger("PathDistCritic"),
      "None of the %d first of %zu (%zu) points of the global plan were in "
      "the local costmap and free",
      i, adjusted_global_plan.poses.size(), global_plan.poses.size());
    return false;
  }
  // 计算路径上各点的曼哈顿距离，用于后续的路径评分
  propogateManhattanDistances();
  //如果一切正常，即全局计划中的前 i 个点中至少有一个点在局部代价地图中是可达且自由的，就返回 true，表示准备工作完成
  return true;
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathDistCritic, dwb_core::TrajectoryCritic)
