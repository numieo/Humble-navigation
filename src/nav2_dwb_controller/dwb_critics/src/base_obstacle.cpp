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
#include <vector>
#include <string>
#include <utility>

#include "dwb_critics/base_obstacle.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::BaseObstacleCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

void BaseObstacleCritic::onInit()
{
  costmap_ = costmap_ros_->getCostmap();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".sum_scores", rclcpp::ParameterValue(false));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".sum_scores", sum_scores_);
}

// 计算给定轨迹的障碍物分数
// const dwb_msgs::msg::Trajectory2D & traj：表示输入的轨迹，其类型
// 是 dwb_msgs::msg::Trajectory2D，包含了一系列姿态信息，描述了机器人的运动轨迹

double BaseObstacleCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  double score = 0.0; // 初始化一个 double 类型的变量 score，用于存储最终的轨迹分数
  // 遍历轨迹中的每个姿态，计算每个姿态的分数，并累加到总分数中
  for (unsigned int i = 0; i < traj.poses.size(); ++i) {
    // 对于轨迹中的每个姿态 traj.poses[i]，调用 scorePose 函数计算该姿态的分数，将结果存储在 pose_score 变量中
    double pose_score = scorePose(traj.poses[i]);
    // Optimized/branchless version of if (sum_scores_) score += pose_score,
    // else score = pose_score;
    score = static_cast<double>(sum_scores_) * score + pose_score;
  }
  return score;
}

// 该函数用于评估给定姿态的障碍物分数
// pose表示输入的姿态，其类型为 geometry_msgs::msg::Pose2D，包含了机器人在二维空间中的位置和朝向信息


double BaseObstacleCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  // 将姿态的世界坐标转换为地图坐标，得到 cell_x 和 cell_y 表示在栅格地图中的单元格坐标
  unsigned int cell_x, cell_y;
  if (!costmap_->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Goes Off Grid.");
  }
  // 获取该单元格的代价值 cost，即表示在该位置的代价信息，通常用于表示障碍物或可行走区域
  unsigned char cost = costmap_->getCost(cell_x, cell_y);
  if (!isValidCost(cost)) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
  }
  return cost;
}

// 判断给定的代价值是否为有效的障碍物代价值
// const unsigned char cost：表示输入的代价值，通常用于表示障碍物的代价信息

bool BaseObstacleCritic::isValidCost(const unsigned char cost)
{
  // 确定代价值是否有效
  // 如果代价值不是这些无效的代价值之一，则被认为是有效的，函数返回 true，否则返回 false
  return cost != nav2_costmap_2d::LETHAL_OBSTACLE &&
         cost != nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
         cost != nav2_costmap_2d::NO_INFORMATION;
}

void BaseObstacleCritic::addCriticVisualization(
  std::vector<std::pair<std::string, std::vector<float>>> & cost_channels)
{
  // // 创建一个存储障碍物评分信息的 pair
  std::pair<std::string, std::vector<float>> grid_scores;
  grid_scores.first = name_;
  // 获取 costmap 的尺寸
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();

  // 调整 vector 大小以容纳全部单元格的评分信息
  grid_scores.second.resize(size_x * size_y);
  unsigned int i = 0;
  for (unsigned int cy = 0; cy < size_y; cy++) {
    for (unsigned int cx = 0; cx < size_x; cx++) {
      // 获取 costmap 在当前单元格的代价值，并将其存储到评分信息的 vector 中
      grid_scores.second[i] = costmap_->getCost(cx, cy);
      i++;
    }
  }
  // 将包含评分信息的 pair 添加到代价通道 vector 中, 以便后续的可视化处理
  cost_channels.push_back(grid_scores);
}

}  // namespace dwb_critics
