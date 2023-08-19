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

#include "dwb_critics/obstacle_footprint.hpp"
#include <algorithm>
#include <vector>
#include "dwb_critics/line_iterator.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::ObstacleFootprintCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{
//根据给定的机器人姿态和足迹规格，计算出机器人足迹在给定姿态下的位置。这在考虑机器人的旋转时，
// 可以用于评估机器人在特定位置的足迹是否与障碍物相交 
// 接受一个机器人的姿态信息和一个足迹规格，然后返回一个在给定姿态下的机器人足迹的位置
Footprint getOrientedFootprint(
  const geometry_msgs::msg::Pose2D & pose,
  const Footprint & footprint_spec)
{
  // 创建一个空的向量，用于存储根据姿态调整后的机器人足迹的位置
  std::vector<geometry_msgs::msg::Point> oriented_footprint;
  // 调整oriented_footprint向量的大小，使其能够容纳与足迹规格中定义的点数相同的点
  oriented_footprint.resize(footprint_spec.size());
  // 计算给定姿态下的余弦和正弦值，用于进行旋转变换
  double cos_th = cos(pose.theta);
  double sin_th = sin(pose.theta);
  // 循环遍历足迹规格中的每个点
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    geometry_msgs::msg::Point & new_pt = oriented_footprint[i];
    // 对于每个点，计算在给定姿态下的新位置
    new_pt.x = pose.x + footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th;
    new_pt.y = pose.y + footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th;
  }
  // 返回已经根据姿态调整的机器人足迹的位置
  return oriented_footprint;
}

// 在规划之前准备机器人的足迹信息，以便后续评估规划轨迹时使用
bool ObstacleFootprintCritic::prepare(
  const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &)
{
  // 从costmap_ros_对象获取机器人的足迹规格，并将其存储在类成员变量footprint_spec_中
  // 足迹规格是一个包含一系列点的列表，这些点描述了机器人的外形
  footprint_spec_ = costmap_ros_->getRobotFootprint();
  if (footprint_spec_.size() == 0) {
    // 如果足迹规格为空，将在日志中记录错误信息，提示可能是由于没有调用setFootprint方法设置足迹规格
    RCLCPP_ERROR(
      rclcpp::get_logger("ObstacleFootprintCritic"),
      "Footprint spec is empty, maybe missing call to setFootprint?");
    return false;
  }
  // 如果足迹规格不为空，将返回true，表示准备工作已经完成
  return true;
}

// 评估给定的机器人姿态下，机器人的足迹与地图中的障碍物是否发生碰撞

double ObstacleFootprintCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  unsigned int cell_x, cell_y;
  // 使用worldToMap函数将世界坐标中的机器人位置转换为地图坐标。如果转换失败，
  // 表示机器人的位置已经超出了地图的范围，这种情况下会抛出一个IllegalTrajectoryException异常，
  // 表示规划轨迹越出了地图的范围
  if (!costmap_->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Goes Off Grid.");
  }
  // 这是scorePose函数的另一个版本的调用，它接受机器人姿态和机器人足迹的方向信息。
  // 这里的目的是计算给定姿态下，机器人足迹与地图中的障碍物之间的碰撞得分
  return scorePose(pose, getOrientedFootprint(pose, footprint_spec_));
}

double ObstacleFootprintCritic::scorePose(
  const geometry_msgs::msg::Pose2D &,
  const Footprint & footprint)
{
  // now we really have to lay down the footprint in the costmap grid
  unsigned int x0, x1, y0, y1;  // 存储足迹中两个相邻点在地图中的坐标
  double line_cost = 0.0;       // 存储线段的代价
  double footprint_cost = 0.0;  // 存储足迹的最大代价

  // 评估机器人足迹与地图中的障碍物之间的碰撞得分
  // we need to rasterize each line in the footprint
  // 循环遍历足迹中相邻的点对，计算每条线段的代价
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    // get the cell coord of the first point
    // 使用 worldToMap 函数将足迹中当前点的世界坐标转换为地图坐标
    // 如果转换失败，表示足迹点超出了地图的范围，这种情况下会抛出一个 
    // IllegalTrajectoryException 异常，表示足迹超出了地图范围
    if (!costmap_->worldToMap(footprint[i].x, footprint[i].y, x0, y0)) {
      throw dwb_core::
            IllegalTrajectoryException(name_, "Footprint Goes Off Grid.");
    }

    // get the cell coord of the second point
    // 将足迹中下一个点的世界坐标转换为地图坐标，如果转换失败，会抛出异常
    if (!costmap_->worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
      throw dwb_core::
            IllegalTrajectoryException(name_, "Footprint Goes Off Grid.");
    }
    // 计算当前线段的代价，lineCost 函数用于计算两个地图坐标之间的线段代价
    line_cost = lineCost(x0, x1, y0, y1);
    // 更新足迹的最大代价，选择当前线段代价和之前的最大代价中的较大值
    footprint_cost = std::max(line_cost, footprint_cost);
  }

  // we also need to connect the first point in the footprint to the last point
  // get the cell coord of the last point
  // 足迹的最后一个点的世界坐标转换为地图坐标。如果转换失败，表示足迹的最后一个点超出了地图的范围，
  // 会抛出一个 IllegalTrajectoryException 异常，表示足迹的最后一个点超出了地图范围
  if (!costmap_->worldToMap(footprint.back().x, footprint.back().y, x0, y0)) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Footprint Goes Off Grid.");
  }

  // get the cell coord of the first point
  // 将足迹的第一个点的世界坐标转换为地图坐标。同样，如果转换失败，会抛出异常
  if (!costmap_->worldToMap(footprint.front().x, footprint.front().y, x1, y1)) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Footprint Goes Off Grid.");
  }
  // 计算足迹首尾两个点之间的线段代价
  line_cost = lineCost(x0, x1, y0, y1);
  // 选择当前线段代价和之前的最大代价中的较大值
  footprint_cost = std::max(line_cost, footprint_cost);

  // if all line costs are legal... then we can return that the footprint is legal
  // 返回足迹的最大代价作为得分
  return footprint_cost;
}

// 计算足迹线段（两个地图坐标之间的连续路径）的碰撞代价

double ObstacleFootprintCritic::lineCost(int x0, int x1, int y0, int y1)
{
  double line_cost = 0.0;     // 初始化线段的总碰撞代价为 0
  double point_cost = -1.0;   // 初始化当前点的碰撞代价为 -1（一个无效值）

  // 使用 LineIterator 对象遍历连接 (x0, y0) 和 (x1, y1) 之间的所有地图单元格
  for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    // 对当前遍历到的地图单元格坐标调用 pointCost 函数，计算当前点的碰撞代价
    point_cost = pointCost(line.getX(), line.getY());   // Score the current point
    // 如果当前点的碰撞代价大于之前记录的线段代价，则更新线段的代价为当前点的代价
    if (line_cost < point_cost) {
      line_cost = point_cost;
    }
  }
  // 返回线段的最大碰撞代价作为结果
  return line_cost;
}

double ObstacleFootprintCritic::pointCost(int x, int y)
{
  // 获取地图上 (x, y) 坐标处的代价值
  unsigned char cost = costmap_->getCost(x, y);
  // if the cell is in an obstacle the path is invalid or unknown
  // 如果该单元格的代价值为致命障碍物（LETHAL_OBSTACLE），则抛出 IllegalTrajectoryException 异常，表示路径碰撞障碍物
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
  } 
  // 如果该单元格的代价值为未知区域（NO_INFORMATION），则抛出 IllegalTrajectoryException 异常，表示路径碰撞未知区域
  else if (cost == nav2_costmap_2d::NO_INFORMATION) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Hits Unknown Region.");
  }
  // 返回单元格的代价值作为结果
  return cost;
}

}  // namespace dwb_critics
