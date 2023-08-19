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

#include "dwb_critics/map_grid.hpp"
#include <cmath>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <memory>
#include "dwb_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"

using std::abs;
using costmap_queue::CellData;

namespace dwb_critics
{

// Customization of the CostmapQueue validCellToQueue method
bool MapGridCritic::MapGridQueue::validCellToQueue(const costmap_queue::CellData & /*cell*/)
{
  return true;
}

void MapGridCritic::onInit()
{
  costmap_ = costmap_ros_->getCostmap();
  queue_ = std::make_shared<MapGridQueue>(*costmap_, *this);

  // Always set to true, but can be overriden by subclasses
  stop_on_failure_ = true;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".aggregation_type",
    rclcpp::ParameterValue(std::string("last")));

  std::string aggro_str;
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".aggregation_type", aggro_str);
  std::transform(aggro_str.begin(), aggro_str.end(), aggro_str.begin(), ::tolower);
  if (aggro_str == "last") {
    aggregationType_ = ScoreAggregationType::Last;
  } else if (aggro_str == "sum") {
    aggregationType_ = ScoreAggregationType::Sum;
  } else if (aggro_str == "product") {
    aggregationType_ = ScoreAggregationType::Product;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "MapGridCritic"), "aggregation_type parameter \"%s\" invalid. Using Last.",
      aggro_str.c_str());
    aggregationType_ = ScoreAggregationType::Last;
  }
}

void MapGridCritic::setAsObstacle(unsigned int index)
{
  cell_values_[index] = obstacle_score_;
}

void MapGridCritic::reset()
{
  queue_->reset();
  cell_values_.resize(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY());
  obstacle_score_ = static_cast<double>(cell_values_.size());
  unreachable_score_ = obstacle_score_ + 1.0;
  std::fill(cell_values_.begin(), cell_values_.end(), unreachable_score_);
}

void MapGridCritic::propogateManhattanDistances()
{
  while (!queue_->isEmpty()) {  // 该循环会一直执行，直到队列不再为空。队列中存储了需要进行距离传播的栅格单元
    costmap_queue::CellData cell = queue_->getNextCell(); // 从队列中获取下一个要处理的栅格单元数据
    // 对当前栅格单元进行距离传播。这里使用曼哈顿距离，计算了当前栅格单元与其邻居之间的绝对差值，并将其加到该栅格单元的cell_values_中
    cell_values_[cell.index_] = CellData::absolute_difference(cell.src_x_, cell.x_) +
      CellData::absolute_difference(cell.src_y_, cell.y_);  
  }
}

// 计算机器人轨迹在代价地图上的得分
double MapGridCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  //  初始化得分为0，该得分将用于计算整个轨迹的代价
  double score = 0.0;
  //  初始化起始索引为0，表示从轨迹的第一个点开始计算代价
  unsigned int start_index = 0;
  // 如果代价聚合类型为乘积（ScoreAggregationType::Product），则将初始得分设置为1.0。这意味着后续将累积乘积代价
  if (aggregationType_ == ScoreAggregationType::Product) {
    score = 1.0;
  } 
  // 如果代价聚合类型为“最后值”（ScoreAggregationType::Last），并且stop_on_failure_为false，
  // 那么将起始索引设置为轨迹的最后一个点的索引。这意味着只计算最后一个点的代价，并忽略之前的点
  else if (aggregationType_ == ScoreAggregationType::Last && !stop_on_failure_) {
    start_index = traj.poses.size() - 1;
  }
  // 用于存储每个轨迹点与代价地图上相应位置的传播距离
  double grid_dist;
  //  从起始索引（根据代价聚合类型和stop_on_failure_的设置）开始循环遍历轨迹中的每个点
  for (unsigned int i = start_index; i < traj.poses.size(); ++i) {
    // 调用scorePose函数计算当前轨迹点在代价地图上的得分，并将得分存储在grid_dist中
    grid_dist = scorePose(traj.poses[i]);
    // 如果stop_on_failure_为true，即在计算过程中遇到代价为障碍或无法到达的情况时停止计算
    if (stop_on_failure_) {
      if (grid_dist == obstacle_score_) {   // 轨迹点遇到了障碍
        throw dwb_core::
              IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
      } else if (grid_dist == unreachable_score_) {   // 表示轨迹点无法到达
        throw dwb_core::
              IllegalTrajectoryException(name_, "Trajectory Hits Unreachable Area.");
      }
    }
    // 根据代价聚合类型更新总得分。根据不同的聚合类型，采取不同的更新策略
    switch (aggregationType_) {
      case ScoreAggregationType::Last: // 将得分设置为当前轨迹点的代价得分
        score = grid_dist;
        break;
      case ScoreAggregationType::Sum:  // 将当前轨迹点的代价得分累加到总得分
        score += grid_dist;
        break;
      case ScoreAggregationType::Product:  // 如果总得分大于0，则将总得分乘以当前轨迹点的代价得分
        if (score > 0) {
          score *= grid_dist;
        }
        break;
    }
  }

  return score;  // 返回计算出的最终得分
}

double MapGridCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  //  声明两个变量用于存储轨迹点在代价地图中的格子索引
  unsigned int cell_x, cell_y;
  // we won't allow trajectories that go off the map... shouldn't happen that often anyways
  // 使用costmap_->worldToMap函数将轨迹点的世界坐标转换为代价地图中的格子索引
  // 如果转换失败，即轨迹点超出了代价地图的范围，抛出IllegalTrajectoryException异常，表示轨迹点越界
  if (!costmap_->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Goes Off Grid.");
  }
  // 传入计算得到的代价地图格子索引，以获取该位置的得分，并将其作为得分返回
  return getScore(cell_x, cell_y);
}

// 实现了在DWBPublisher中将MapGridCritic的得分信息可视化的操作

void MapGridCritic::addCriticVisualization(
  std::vector<std::pair<std::string, std::vector<float>>> & cost_channels)
{
  //  创建一个pair，用于存储表示地图网格批评者的名称和得分数据
  std::pair<std::string, std::vector<float>> grid_scores;
  // 将地图网格批评者的名称设置为grid_scores的第一个元素，这是一个字符串
  grid_scores.first = name_;
  // 获取代价地图
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  // 获取代价地图的大小，即X轴和Y轴上的单元格数量
  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();
  // 调整grid_scores的第二个元素，即存储得分的向量，使其具有足够的大小以容纳整个代价地图
  grid_scores.second.resize(size_x * size_y);
  unsigned int i = 0;
  // 遍历代价地图中的每个单元格：
  for (unsigned int cy = 0; cy < size_y; cy++) {
    for (unsigned int cx = 0; cx < size_x; cx++) {
      // 在每个单元格上调用getScore(cx, cy)，并将结果存储在grid_scores.second[i]中，
      // 其中i是代表当前单元格在向量中的索引
      grid_scores.second[i] = getScore(cx, cy);
      i++;
    }
  }
  // 将填充了得分数据的grid_scores添加到cost_channels向量中，这将用于发布可视化信息
  cost_channels.push_back(grid_scores);
}

}  // namespace dwb_critics
