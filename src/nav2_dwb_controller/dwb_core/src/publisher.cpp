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

#include "dwb_core/publisher.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav2_util/node_utils.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::max;
using std::string;
using nav2_util::declare_parameter_if_not_declared;

namespace dwb_core
{

DWBPublisher::DWBPublisher(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
: node_(parent),
  plugin_name_(plugin_name)
{
  auto node = node_.lock();
  clock_ = node->get_clock();
}

nav2_util::CallbackReturn
DWBPublisher::on_configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_evaluation",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_global_plan",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_transformed_plan",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_local_plan",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_trajectories",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_cost_grid_pc",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".marker_lifetime",
    rclcpp::ParameterValue(0.1));

  node->get_parameter(plugin_name_ + ".publish_evaluation", publish_evaluation_);
  node->get_parameter(plugin_name_ + ".publish_global_plan", publish_global_plan_);
  node->get_parameter(plugin_name_ + ".publish_transformed_plan", publish_transformed_);
  node->get_parameter(plugin_name_ + ".publish_local_plan", publish_local_plan_);
  node->get_parameter(plugin_name_ + ".publish_trajectories", publish_trajectories_);
  node->get_parameter(plugin_name_ + ".publish_cost_grid_pc", publish_cost_grid_pc_);

  eval_pub_ = node->create_publisher<dwb_msgs::msg::LocalPlanEvaluation>("evaluation", 1);
  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  transformed_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);
  local_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 1);
  cost_grid_pc_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("cost_cloud", 1);

  double marker_lifetime = 0.0;
  node->get_parameter(plugin_name_ + ".marker_lifetime", marker_lifetime);
  marker_lifetime_ = rclcpp::Duration::from_seconds(marker_lifetime);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DWBPublisher::on_activate()
{
  eval_pub_->on_activate();
  global_pub_->on_activate();
  transformed_pub_->on_activate();
  local_pub_->on_activate();
  marker_pub_->on_activate();
  cost_grid_pc_pub_->on_activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DWBPublisher::on_deactivate()
{
  eval_pub_->on_deactivate();
  global_pub_->on_deactivate();
  transformed_pub_->on_deactivate();
  local_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  cost_grid_pc_pub_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DWBPublisher::on_cleanup()
{
  eval_pub_.reset();
  global_pub_.reset();
  transformed_pub_.reset();
  local_pub_.reset();
  marker_pub_.reset();
  cost_grid_pc_pub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

void        // 发布路径规划的评估结果   // results 作为参数，其中包含了路径规划的评估信息
DWBPublisher::publishEvaluation(std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results)
{
  if (results) {  //  检查 results 是否为非空
    // 检查是否需要发布评估信息，并且当前是否有订阅者对评估信息感兴趣
    // publish_evaluation_: 一个标志，指示是否应该发布评估信息
    // eval_pub_->get_subscription_count() > 0: 检查评估信息的发布者是否有订阅者
    if (publish_evaluation_ && eval_pub_->get_subscription_count() > 0) {
      // 使用参数中的评估结果初始化
      auto msg = std::make_unique<dwb_msgs::msg::LocalPlanEvaluation>(*results);
      // 检查评估信息的发布者是否有订阅
      eval_pub_->publish(std::move(msg));
    }
    // 调用另一个函数 publishTrajectories 来发布轨迹相关的信息，传递参数中的评估结果
    publishTrajectories(*results);
  }
}

// 发布轨迹信息的可视化效果，接受一个 dwb_msgs::msg::LocalPlanEvaluation 类型的参数 results，包含路径规划的评估信息

void
DWBPublisher::publishTrajectories(const dwb_msgs::msg::LocalPlanEvaluation & results)
{
  // 检查轨迹可视化的发布者是否有订阅者，如果没有订阅者则返回
  if (marker_pub_->get_subscription_count() < 1) {return;}
  // 检查是否需要发布轨迹信息，如果不需要则返回
  if (!publish_trajectories_) {return;}
  // 创建一个新的 visualization_msgs::msg::MarkerArray 消息用于存储轨迹可视化信息
  auto ma = std::make_unique<visualization_msgs::msg::MarkerArray>();
  visualization_msgs::msg::Marker m;
  // 检查是否需要发布轨迹信息，如果不需要则返回
  if (results.twists.size() == 0) {return;}

  geometry_msgs::msg::Point pt;

  m.header = results.header;     // 设置标记的头部信息
  m.type = m.LINE_STRIP;         // 设置标记类型为线条
  m.pose.orientation.w = 1;      // 设置标记的方向
  m.scale.x = 0.002;             // 设置标记的线条宽度
  m.color.a = 1.0;               // 设置标记的颜色透明度
  m.lifetime = marker_lifetime_; // 设置标记的生命周期

  double best_cost = results.twists[results.best_index].total;    // 获取最佳轨迹的总成本
  double worst_cost = results.twists[results.worst_index].total;  // 获取最差轨迹的总成本
  double denominator = worst_cost - best_cost;                    // 计算最差轨迹和最佳轨迹总成本之差

  // 如果差的绝对值小于一个较小的阈值，将差设置为1.0，避免除以零
  if (std::fabs(denominator) < 1e-9) {
    denominator = 1.0;
  }

  // 初始化两个计数器，用于为不同类型的轨迹分配唯一的ID
  unsigned currentValidId = 0;
  unsigned currentInvalidId = 0;
  // 定义两个命名空间，用于区分合法和非法轨迹的显示
  string validNamespace("ValidTrajectories");
  string invalidNamespace("InvalidTrajectories");
  // 循环遍历每个轨迹评估结果
  for (unsigned int i = 0; i < results.twists.size(); i++) {
    const dwb_msgs::msg::TrajectoryScore & twist = results.twists[i]; // 获取第i个轨迹评估结果
    double displayLevel = (twist.total - best_cost) / denominator;    // 计算显示级别，用于确定颜色的变化程度
    // 判断轨迹是否合法
    // 如果 twist.total 大于等于0，表示轨迹合法，则设置标记的颜色为根据 displayLevel 计算的红色和绿色的组合，以及蓝色为0。将标记的命名空间设置为 validNamespace，并分配唯一的ID
    if (twist.total >= 0) {
      m.color.r = displayLevel;
      m.color.g = 1.0 - displayLevel;
      m.color.b = 0;
      m.color.a = 1.0;
      m.ns = validNamespace;
      m.id = currentValidId;
      ++currentValidId;
    }
    // 如果 twist.total 小于0，表示轨迹非法，则设置标记的颜色为黑色。将标记的命名空间设置为 invalidNamespace，并分配唯一的ID
     else {
      m.color.r = 0;
      m.color.g = 0;
      m.color.b = 0;
      m.color.a = 1.0;
      m.ns = invalidNamespace;
      m.id = currentInvalidId;
      ++currentInvalidId;
    }
    // 清除上一轨迹的点信息
    m.points.clear();

    // 遍历当前轨迹的每个点，并将其添加到标记的点列表中
    for (unsigned int j = 0; j < twist.traj.poses.size(); ++j) {
      pt.x = twist.traj.poses[j].x;
      pt.y = twist.traj.poses[j].y;
      pt.z = 0;
      m.points.push_back(pt);
    }
    // 将标记添加到 visualization_msgs::msg::MarkerArray 中
    ma->markers.push_back(m);
  }
  // 发布包含轨迹可视化信息
  marker_pub_->publish(std::move(ma));
}

void  // 发布局部轨迹计划的可视化
DWBPublisher::publishLocalPlan(
  const std_msgs::msg::Header & header,
  const dwb_msgs::msg::Trajectory2D & traj)
{
  // 如果配置中不允许发布局部轨迹计划，那么直接返回，不进行后续操作
  if (!publish_local_plan_) {return;}
  // 将输入的 dwb_msgs::msg::Trajectory2D 转换为 nav_msgs::msg::Path 消息
  auto path =
    std::make_unique<nav_msgs::msg::Path>(
    // 使用 nav_2d_utils::poses2DToPath 函数将轨迹中的位姿信息转换为路径信息，同时指定了路径的坐标系和时间戳
    nav_2d_utils::poses2DToPath(
      traj.poses, header.frame_id,
      header.stamp));
  // 如果有节点订阅局部轨迹计划的话，就将转换后的 nav_msgs::msg::Path 消息发布出去。
  // 这可以让其他ROS节点监听并显示局部轨迹计划的可视化结果
  if (local_pub_->get_subscription_count() > 0) {
    local_pub_->publish(std::move(path));
  }
}

// 用于发布成本地图的可视化。成本地图是一个表示环境中各个点的代价值的二维数据结构

void
DWBPublisher::publishCostGrid(
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  const std::vector<TrajectoryCritic::Ptr> critics)
{
  // 如果没有节点订阅成本地图的可视化，就直接返回，不进行后续操作
  if (cost_grid_pc_pub_->get_subscription_count() < 1) {return;}
  // 如果配置中不允许发布成本地图，也直接返回，不进行后续操作
  if (!publish_cost_grid_pc_) {return;}
  // 用于存储成本地图的可视化数据
  auto cost_grid_pc = std::make_unique<sensor_msgs::msg::PointCloud2>();
  // 设置消息的坐标系，通常是全局坐标系
  cost_grid_pc->header.frame_id = costmap_ros->getGlobalFrameID(); 
  // 设置消息的时间戳为当前时间
  cost_grid_pc->header.stamp = clock_->now();
  //  获取导航成本地图的指针
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros->getCostmap();
  // 获取成本地图的尺寸
  double x_coord, y_coord;
  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();
  // 存储不同代价通道的信息
  std::vector<std::pair<std::string, std::vector<float>>> cost_channels;
  // 存储每个格子的代价值
  std::vector<float> total_cost(size_x * size_y, 0.0);
  // 对于每个批判者（critic）进行迭代，将他们的代价数据添加到用于成本地图可视化的消息中
  for (TrajectoryCritic::Ptr critic : critics) {
    // 获取当前代价通道的索引
    unsigned int channel_index = cost_channels.size();
    // 调用批判者（critic）的方法，将代价数据添加到 cost_channels 中
    critic->addCriticVisualization(cost_channels);
    // 如果没有新的代价通道被添加，说明该批判者（critic）没有为可视化提供新的数据，那么跳过这个批判者的处理
    if (channel_index == cost_channels.size()) {
      // No channels were added, so skip to next critic
      continue;
    }
    // 获取批判者（critic）的缩放因子
    double scale = critic->getScale();
    //  遍历每个格子，将批判者（critic）的代价数据乘以缩放因子并添加到总代价中
    for (unsigned int i = 0; i < size_x * size_y; i++) {
      total_cost[i] += cost_channels[channel_index].second[i] * scale;
    }
  }

  // 将总代价通道的数据添加到 cost_channels 中
  cost_channels.push_back(std::make_pair("total_cost", total_cost));
  // 设置 cost_grid_pc 的宽度为 size_x * size_y，高度为 1，字段数量为 3（x、y、z坐标）加上代价通道的数量
  cost_grid_pc->width = size_x * size_y;
  cost_grid_pc->height = 1;
  cost_grid_pc->fields.resize(3 + cost_channels.size());  // x,y,z, + cost channels
  //  设置 cost_grid_pc 的属性，表示数据是稠密的，且不使用大端字节顺序
  cost_grid_pc->is_dense = true;
  cost_grid_pc->is_bigendian = false;

  int offset = 0;   // 初始化偏移量为 0
  // 遍历 cost_grid_pc 的字段数组，为每个字段设置属性。offset 在每次迭代中都会递增 4，因为每个字段的大小是 4 字节（FLOAT32
  for (size_t i = 0; i < cost_grid_pc->fields.size(); ++i, offset += 4) {
    cost_grid_pc->fields[i].offset = offset;    // 设置字段的偏移
    cost_grid_pc->fields[i].count = 1;          // 设置字段的数量
    // 设置字段的数据类型为 FLOAT32
    cost_grid_pc->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    //  如果字段的索引大于等于 3（即代价通道字段），则将该字段的名称设置为相应的代价通道名称
    if (i >= 3) {
      cost_grid_pc->fields[i].name = cost_channels[i - 3].first;
    }
  }
  // 将前三个字段（x、y、z坐标）的名称设置为 "x"、"y" 和 "z"
  cost_grid_pc->fields[0].name = "x";
  cost_grid_pc->fields[1].name = "y";
  cost_grid_pc->fields[2].name = "z";
  // 设置每个点的大小，即 point_step，等于 offset
  cost_grid_pc->point_step = offset;
  // 设置每一行的大小，即 row_step，等于 point_step 乘以消息的宽度
  cost_grid_pc->row_step = cost_grid_pc->point_step * cost_grid_pc->width;
  // 分配数据内存，大小为 row_step 乘以消息的高度。这将为消息的数据预留足够的空间
  cost_grid_pc->data.resize(cost_grid_pc->row_step * cost_grid_pc->height);
  //  创建一个存储 sensor_msgs::PointCloud2Iterator<float> 迭代器的向量，用于遍历点云消息中的每个字段
  std::vector<sensor_msgs::PointCloud2Iterator<float>> cost_grid_pc_iter;
  // 遍历代价图点云消息的字段数组，为每个字段创建相应的 PointCloud2Iterator<float> 迭代器，并将其存储在 cost_grid_pc_iter 中
  for (size_t i = 0; i < cost_grid_pc->fields.size(); ++i) {
    sensor_msgs::PointCloud2Iterator<float> iter(*cost_grid_pc, cost_grid_pc->fields[i].name);
    cost_grid_pc_iter.push_back(iter);
  }
  // 循环遍历代价图的单元格，将代价信息填充到点云消息中
  unsigned int j = 0;
  for (unsigned int cy = 0; cy < size_y; cy++) {
    for (unsigned int cx = 0; cx < size_x; cx++) {
        // 将代价图的单元格坐标转换为世界坐标
      costmap->mapToWorld(cx, cy, x_coord, y_coord);  // 将代价图的单元格坐标转换为世界坐标
      *cost_grid_pc_iter[0] = x_coord;  // 将 x 坐标写入点云消息的 x 字段
      *cost_grid_pc_iter[1] = y_coord;  // 将 y 坐标写入点云消息的 y 字段
      *cost_grid_pc_iter[2] = 0.0;   // z value
      // 遍历除了前三个坐标字段之外的所有字段
      for (size_t i = 3; i < cost_grid_pc_iter.size(); ++i) {
        // 将对应代价通道的值写入点云消息的字段中
        *cost_grid_pc_iter[i] = cost_channels[i - 3].second[j];
        ++cost_grid_pc_iter[i];   // 将迭代器指向下一个单元格，为下一次循环做准备
      }
      ++cost_grid_pc_iter[0];     // 将 x 坐标字段的迭代器指向下一个单元格
      ++cost_grid_pc_iter[1];     // 将 y 坐标字段的迭代器指向下一个单元格
      ++cost_grid_pc_iter[2];     // 将 z 坐标字段的迭代器指向下一个单元格
      j++;                  // 增加索引 j，用于在代价通道数据中获取下一个值
    }
  }
  // 将填充好代价数据的点云消息发布出去
  cost_grid_pc_pub_->publish(std::move(cost_grid_pc));
}

// 将传入的全局计划 plan 以及发布相关标志 publish_global_plan_ 传递给 global_pub_ 来发布全局计划的信息

void
DWBPublisher::publishGlobalPlan(const nav_2d_msgs::msg::Path2D plan)
{
  publishGenericPlan(plan, *global_pub_, publish_global_plan_);
}

// 通过调用 publishGenericPlan 函数，将传入的转换后的计划 plan 以及发布相关标志 publish_transformed_ 传递给 transformed_pub_ 来发布转换后的计划的信息

void
DWBPublisher::publishTransformedPlan(const nav_2d_msgs::msg::Path2D plan)
{
  publishGenericPlan(plan, *transformed_pub_, publish_transformed_);
}

// 将传入的本地计划 plan 以及发布相关标志 publish_local_plan_ 传递给 local_pub_ 来发布本地计划的信息

void
DWBPublisher::publishLocalPlan(const nav_2d_msgs::msg::Path2D plan)
{
  publishGenericPlan(plan, *local_pub_, publish_local_plan_);
}

void
DWBPublisher::publishGenericPlan(
  const nav_2d_msgs::msg::Path2D plan,
  rclcpp::Publisher<nav_msgs::msg::Path> & pub, bool flag)
{
  // 检查发布者的订阅者数量是否小于1，如果没有订阅者，就不会执行发布操作，以避免不必要的计算和资源浪费
  if (pub.get_subscription_count() < 1) {return;}
  // 函数会检查标志 flag 的值，如果为 false，也不会执行发布操作，以便在需要时控制是否发布计划信息
  if (!flag) {return;}
  // 将传入的 nav_2d_msgs::msg::Path2D 类型的计划路径 plan 转换为 nav_msgs::msg::Path 类型的路径消息，并通过 pub 发布到相应的话题
  auto path = std::make_unique<nav_msgs::msg::Path>(nav_2d_utils::pathToPath(plan));
  pub.publish(std::move(path));
}

}  // namespace dwb_core
