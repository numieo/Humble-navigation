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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dwb_core/dwb_local_planner.hpp"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "dwb_msgs/msg/critic_score.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace dwb_core
{

DWBLocalPlanner::DWBLocalPlanner()
: traj_gen_loader_("dwb_core", "dwb_core::TrajectoryGenerator"),
  critic_loader_("dwb_core", "dwb_core::TrajectoryCritic")
{
}

void DWBLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();

  logger_ = node->get_logger();
  clock_ = node->get_clock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  dwb_plugin_name_ = name;
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".critics",
    rclcpp::PARAMETER_STRING_ARRAY);
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".default_critic_namespaces",
    rclcpp::ParameterValue(std::vector<std::string>()));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".prune_plan",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".prune_distance",
    rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".forward_prune_distance",
    rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".debug_trajectory_details",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".trajectory_generator_name",
    rclcpp::ParameterValue(std::string("dwb_plugins::StandardTrajectoryGenerator")));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".transform_tolerance",
    rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".shorten_transformed_plan",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".short_circuit_trajectory_evaluation",
    rclcpp::ParameterValue(true));

  std::string traj_generator_name;

  double transform_tolerance;
  node->get_parameter(dwb_plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
  RCLCPP_INFO(logger_, "Setting transform_tolerance to %f", transform_tolerance);

  node->get_parameter(dwb_plugin_name_ + ".prune_plan", prune_plan_);
  node->get_parameter(dwb_plugin_name_ + ".prune_distance", prune_distance_);
  node->get_parameter(dwb_plugin_name_ + ".forward_prune_distance", forward_prune_distance_);
  node->get_parameter(dwb_plugin_name_ + ".debug_trajectory_details", debug_trajectory_details_);
  node->get_parameter(dwb_plugin_name_ + ".trajectory_generator_name", traj_generator_name);
  node->get_parameter(
    dwb_plugin_name_ + ".short_circuit_trajectory_evaluation",
    short_circuit_trajectory_evaluation_);
  node->get_parameter(dwb_plugin_name_ + ".shorten_transformed_plan", shorten_transformed_plan_);

  pub_ = std::make_unique<DWBPublisher>(node, dwb_plugin_name_);
  pub_->on_configure();

  traj_generator_ = traj_gen_loader_.createUniqueInstance(traj_generator_name);

  traj_generator_->initialize(node, dwb_plugin_name_);

  try {
    loadCritics();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Couldn't load critics! Caught exception: %s", e.what());
    throw;
  }
}

void
DWBLocalPlanner::activate()
{
  pub_->on_activate();
}

void
DWBLocalPlanner::deactivate()
{
  pub_->on_deactivate();
}

void
DWBLocalPlanner::cleanup()
{
  pub_->on_cleanup();

  traj_generator_.reset();
}

std::string
DWBLocalPlanner::resolveCriticClassName(std::string base_name)
{
  if (base_name.find("Critic") == std::string::npos) {
    base_name = base_name + "Critic";
  }

  if (base_name.find("::") == std::string::npos) {
    for (unsigned int j = 0; j < default_critic_namespaces_.size(); j++) {
      std::string full_name = default_critic_namespaces_[j] + "::" + base_name;
      if (critic_loader_.isClassAvailable(full_name)) {
        return full_name;
      }
    }
  }
  return base_name;
}

void
DWBLocalPlanner::loadCritics()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(dwb_plugin_name_ + ".default_critic_namespaces", default_critic_namespaces_);
  if (default_critic_namespaces_.empty()) {
    default_critic_namespaces_.emplace_back("dwb_critics");
  }

  std::vector<std::string> critic_names;
  if (!node->get_parameter(dwb_plugin_name_ + ".critics", critic_names)) {
    throw std::runtime_error("No critics defined for " + dwb_plugin_name_);
  }

  node->get_parameter(dwb_plugin_name_ + ".critics", critic_names);
  for (unsigned int i = 0; i < critic_names.size(); i++) {
    std::string critic_plugin_name = critic_names[i];
    std::string plugin_class;

    declare_parameter_if_not_declared(
      node, dwb_plugin_name_ + "." + critic_plugin_name + ".class",
      rclcpp::ParameterValue(critic_plugin_name));
    node->get_parameter(dwb_plugin_name_ + "." + critic_plugin_name + ".class", plugin_class);

    plugin_class = resolveCriticClassName(plugin_class);

    TrajectoryCritic::Ptr plugin = critic_loader_.createUniqueInstance(plugin_class);
    RCLCPP_INFO(
      logger_,
      "Using critic \"%s\" (%s)", critic_plugin_name.c_str(), plugin_class.c_str());
    critics_.push_back(plugin);
    try {
      plugin->initialize(node, critic_plugin_name, dwb_plugin_name_, costmap_ros_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Couldn't initialize critic plugin!");
      throw;
    }
    RCLCPP_INFO(logger_, "Critic plugin initialized");
  }
}

void    // 此函数用于设置规划的全局路径。它将接收到的nav_msgs::msg::Path路径转换为2D路径，并初始化路径规划器的各项参数
DWBLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  // 通过nav_2d_utils::pathToPath2D(path)将全局路径转换为2D路径
  auto path2d = nav_2d_utils::pathToPath2D(path);
  // 遍历批评家（critics_），将它们的状态重置为初始状态，以准备计算路径的代价
  for (TrajectoryCritic::Ptr & critic : critics_) {
    critic->reset();
  }
  // 调用traj_generator_->reset()重置轨迹生成器，以准备生成新的轨迹
  traj_generator_->reset();
  // 发布全局路径以供调试和可视化，并将路径存储在global_plan_变量中
  pub_->publishGlobalPlan(path2d);
  global_plan_ = path2d;
}
// 计算在给定机器人位姿、当前速度和目标检查器的情况下，机器人的下一个速度指令，会尝试计算速度指令，同时记录路径规划评估信息
geometry_msgs::msg::TwistStamped
DWBLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results = nullptr;
  // 如果需要记录评估信息（根据pub_->shouldRecordEvaluation()），则创建一个dwb_msgs::msg::LocalPlanEvaluation实例以存储评估结果
  if (pub_->shouldRecordEvaluation()) {
    results = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
  }
  // 调用computeVelocityCommands函数进行实际的速度计算，传递2D位姿和速度，并将评估结果存储在results中
  try {
    nav_2d_msgs::msg::Twist2DStamped cmd_vel2d = computeVelocityCommands(
      nav_2d_utils::poseStampedToPose2D(pose),
      nav_2d_utils::twist3Dto2D(velocity), results);
    // 通过pub_->publishEvaluation(results)将评估结果发布出去
    pub_->publishEvaluation(results);
    geometry_msgs::msg::TwistStamped cmd_vel;
    // 将2D速度指令转换为3D速度指令，并将其包装在geometry_msgs::msg::TwistStamped中返回
    cmd_vel.twist = nav_2d_utils::twist2Dto3D(cmd_vel2d.velocity);
    return cmd_vel;
  } catch (const nav2_core::PlannerException & e) {
    pub_->publishEvaluation(results);
    throw;
  }
}
/**
 * @brief 用于准备全局路径，将其转换并发布变换后的路径和目标位姿
 * 
 * @param pose 机器人当前位姿，用于进行路径的坐标变换
 * @param transformed_plan 存储变换后的全局路径
 * @param goal_pose 存储目标位姿，即全局路径的最后一个点
 * @param publish_plan 表示是否要发布变换后的路径
 */
void
DWBLocalPlanner::prepareGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose, nav_2d_msgs::msg::Path2D & transformed_plan,
  nav_2d_msgs::msg::Pose2DStamped & goal_pose, bool publish_plan)
{
  // 调用transformGlobalPlan(pose)将全局路径进行坐标变换，并将结果存储在transformed_plan中
  transformed_plan = transformGlobalPlan(pose);
  if (publish_plan) {
    pub_->publishTransformedPlan(transformed_plan); // 将变换后的全局路径进行发布
  }
  // 设置goal_pose的头部帧ID为全局路径的帧ID，并将目标位姿设置为全局路径的最后一个点
  goal_pose.header.frame_id = global_plan_.header.frame_id;
  goal_pose.pose = global_plan_.poses.back();
  // 将目标位姿进行坐标变换，以便在指定坐标系中表示
  nav_2d_utils::transformPose(
    tf_, costmap_ros_->getGlobalFrameID(), goal_pose,
    goal_pose, transform_tolerance_);
}

/**
 * @brief 计算机器人的期望速度指令
 * 
 * @param pose 机器人当前位姿
 * @param velocity 当前机器人的线速度和角速度
 * @param results 存储评估结果
 * @return nav_2d_msgs::msg::Twist2DStamped 机器人的期望速度指令
 */
nav_2d_msgs::msg::Twist2DStamped
DWBLocalPlanner::computeVelocityCommands(
  const nav_2d_msgs::msg::Pose2DStamped & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  // 如果results不为空，设置results消息的头部帧ID为传入的位姿的帧ID，时间戳为当前时间
  if (results) {
    results->header.frame_id = pose.header.frame_id;
    results->header.stamp = clock_->now();
  }

  nav_2d_msgs::msg::Path2D transformed_plan;
  nav_2d_msgs::msg::Pose2DStamped goal_pose;
  // pose 参数是机器人的当前位姿，transformed_plan 是一个引用，用于存储转换后的全局路径，goal_pose 是一个引用，用于存储目标位姿。
  // publish_plan 是一个布尔值，表示是否在转换后的全局路径进行发布。
  // 首先，函数将调用 transformGlobalPlan 函数对全局路径进行转换，然后根据传入的 publish_plan 值决定是否发布转换后的全局路径。
  // 接下来，函数设置目标位姿的帧 ID 为全局路径的帧 ID，将目标位姿设置为全局路径中的最后一个位姿。
  // 最后，使用 nav_2d_utils::transformPose 函数将目标位姿转换到代价地图的全局坐标系中，根据 transform_tolerance_ 参数进行坐标变换。
  prepareGlobalPlan(pose, transformed_plan, goal_pose);
  // 获取代价地图的指针，并在获取锁的情况下进行操作
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  // 遍历路径评价器（critics），对每个评价器执行准备操作，传入当前位姿、速度、目标位姿和准备好的全局路径
  for (TrajectoryCritic::Ptr & critic : critics_) {
    // 如果评价器的prepare操作失败，在日志中发出警告
    if (!critic->prepare(pose.pose, velocity, goal_pose.pose, transformed_plan)) {
      RCLCPP_WARN(rclcpp::get_logger("DWBLocalPlanner"), "A scoring function failed to prepare");
    }
  }

  try {
    // 计算得分最高的轨迹
    dwb_msgs::msg::TrajectoryScore best = coreScoringAlgorithm(pose.pose, velocity, results);

    // Return Value
    // 根据 best 变量中存储的最佳轨迹，构建一个包含时间戳和速度的 nav_2d_msgs::msg::Twist2DStamped 消息，并返回给调用者
    nav_2d_msgs::msg::Twist2DStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.velocity = best.traj.velocity;

    // debrief stateful scoring functions
    for (TrajectoryCritic::Ptr & critic : critics_) {
      critic->debrief(cmd_vel.velocity);
    }
    // 对每个轨迹评分函数（critics_ 中的每个批评者）进行“总结”（debrief），以进行状态清理或记录
    lock.unlock();
    // 使用 pub_ 对象将计算得到的最佳轨迹发布为局部规划，同时也发布代价地图和评分信息
    pub_->publishLocalPlan(pose.header, best.traj);
    pub_->publishCostGrid(costmap_ros_, critics_);

    return cmd_vel;
  } catch (const dwb_core::NoLegalTrajectoriesException & e) { // 表示找不到合法的轨迹
    nav_2d_msgs::msg::Twist2D empty_cmd;  // 表示没有可行的速度指令
    dwb_msgs::msg::Trajectory2D empty_traj;  // 表示没有可行的轨迹
    // debrief stateful scoring functions
    for (TrajectoryCritic::Ptr & critic : critics_) {  // 循环遍历所有的轨迹评分函数
      critic->debrief(empty_cmd);   // 调用每个评分函数的 debrief 方法，将空的速度指令传递给它们，以清理评分函数的状态
    }

    lock.unlock();      // 解锁代价地图的互斥锁，以允许其他线程访问代价地图
    // 使用 pub_ 对象发布空的局部规划（empty_traj）和代价地图以及评分信息
    pub_->publishLocalPlan(pose.header, empty_traj);
    pub_->publishCostGrid(costmap_ros_, critics_);
    // 将捕获到的异常重新抛出，以便外部代码能够处理这个异常情况
    throw;
  }
}

dwb_msgs::msg::TrajectoryScore
DWBLocalPlanner::coreScoringAlgorithm(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  nav_2d_msgs::msg::Twist2D twist;            // 用于存储当前的速度指令
  dwb_msgs::msg::Trajectory2D traj;           // 存储生成的轨迹
  dwb_msgs::msg::TrajectoryScore best, worst; // 存储最佳和最差的轨迹评分
  best.total = -1;                            // 确保在迭代过程中可以正确地更新最佳和最差评分
  worst.total = -1;
  IllegalTrajectoryTracker tracker;           // 开始一个新的迭代，该迭代将生成与给定速度指令相关的一系列轨迹
  
  // 开始一个新的迭代，该迭代将生成与给定速度指令相关的一系列轨迹
  traj_generator_->startNewIteration(velocity);
  while (traj_generator_->hasMoreTwists()) {  // 直到没有更多的速度指令可用
    twist = traj_generator_->nextTwist();     // 获取下一个速度指令 twist
    // 使用当前的姿态、速度和速度指令 生成一个轨迹 traj
    traj = traj_generator_->generateTrajectory(pose, velocity, twist);

    try {
      // 调用 scoreTrajectory 函数来对当前生成的轨迹 traj 进行评分，同时提供了当前最佳评分 best.total 作为参考
      dwb_msgs::msg::TrajectoryScore score = scoreTrajectory(traj, best.total);
      // 调用 tracker.addLegalTrajectory() 来记录生成的轨迹是合法的，以便后续跟踪
      tracker.addLegalTrajectory();
      // 如果传入的 results 不为空，则将当前轨迹的评分信息添加到 results->twists 中
      if (results) {
        results->twists.push_back(score);
      }
      // 如果当前轨迹的总分 score.total 比之前的最佳评分 best.total 更好，
      // 更新 best 的值为当前轨迹的评分，并在 results 中记录最佳轨迹的索引
      if (best.total < 0 || score.total < best.total) {
        best = score;
        if (results) {
          results->best_index = results->twists.size() - 1;
        }
      }
      // 如果当前轨迹的总分 score.total 比之前的最差评分 worst.total 更差，
      // 更新 worst 的值为当前轨迹的评分，并在 results 中记录最差轨迹的索引
      if (worst.total < 0 || score.total > worst.total) {
        worst = score;
        if (results) {
          results->worst_index = results->twists.size() - 1;
        }
      }
    } catch (const dwb_core::IllegalTrajectoryException & e) {
      // 如果传入的 results 不为空，则创建一个 failed_score 对象，将不合法的轨迹信息（traj）保存在其中
      if (results) {
        dwb_msgs::msg::TrajectoryScore failed_score;
        failed_score.traj = traj;
        // 记录导致轨迹不合法的批评者（critic）的名称，并将原始评分设置为 -1.0
        dwb_msgs::msg::CriticScore cs;
        cs.name = e.getCriticName();
        cs.raw_score = -1.0;          // 表示轨迹总分无效
        failed_score.scores.push_back(cs);
        failed_score.total = -1.0;    // 将 failed_score 添加到 results->twists 中，以便记录不合法的轨迹信息
        results->twists.push_back(failed_score);
      }
      // 跟踪记录不合法的轨迹信息，其中的异常对象 e 包含有关不合法情况的详细信息
      tracker.addIllegalTrajectory(e);
    }
  }

  if (best.total < 0) {         // 表示在生成的所有轨迹中都没有找到合法的轨迹
    if (debug_trajectory_details_) {
      // 输出关于不合法轨迹的详细信息和各个轨迹批评者的百分比，有助于调试和了解为什么没有找到合法的轨迹
      RCLCPP_ERROR(rclcpp::get_logger("DWBLocalPlanner"), "%s", tracker.getMessage().c_str());
      for (auto const & x : tracker.getPercentages()) {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "DWBLocalPlanner"), "%.2f: %10s/%s", x.second,
          x.first.first.c_str(), x.first.second.c_str());
      }
    }
    throw NoLegalTrajectoriesException(tracker);  // 这表示找不到合法的轨迹
  }

  return best;    // 如果找到了合法的轨迹，则返回 best，它是评分最高的合法轨迹
}

dwb_msgs::msg::TrajectoryScore
DWBLocalPlanner::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & traj,   // 待评分的轨迹 traj
  double best_score)                          // 已经计算得到的最佳得分 best_score
{
  dwb_msgs::msg::TrajectoryScore score;       // 用于存储轨迹的得分和其他相关信息
  score.traj = traj;
  // 对每一个注册的评价标准
  for (TrajectoryCritic::Ptr & critic : critics_) {
    dwb_msgs::msg::CriticScore cs;  // 用于存储评价标准的得分和其他信息
    cs.name = critic->getName();    // 获取评价标准的名称
    cs.scale = critic->getScale();  // 缩放因子

    // 如果缩放因子为零，表示该评价标准不会对轨迹得分产生影响
    if (cs.scale == 0.0) {
      score.scores.push_back(cs);
      continue;
    }
    // 将 cs 添加到 score.scores 并跳过当前循环
    double critic_score = critic->scoreTrajectory(traj);
    cs.raw_score = critic_score;  // 将原始得分添加到 cs 的 raw_score 字段中
    score.scores.push_back(cs); 
    score.total += critic_score * cs.scale;
    // 如果开启了 short_circuit_trajectory_evaluation_，并且最佳得分 best_score 大于零，
    // 并且当前得分 score.total 超过了最佳得分，就会中断轨迹的进一步评估
    if (short_circuit_trajectory_evaluation_ && best_score > 0 && score.total > best_score) {
      // since we keep adding positives, once we are worse than the best, we will stay worse
      break;
    }
  }
  // 返回最终的轨迹得分
  return score;
}

nav_2d_msgs::msg::Path2D
DWBLocalPlanner::transformGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose)
{
  if (global_plan_.poses.empty()) {
    // 代码检查全局路径 global_plan_ 是否为空，如果为空，表示收到了一个零长度的路径
    // 抛出异常 nav2_core::PlannerException
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  nav_2d_msgs::msg::Pose2DStamped robot_pose;
  // 通过调用 nav_2d_utils::transformPose 函数，将输入的机器人位姿 pose 从全局路径的坐标系转换到机器人的坐标系
  if (!nav_2d_utils::transformPose(
      tf_, global_plan_.header.frame_id, pose,
      robot_pose, transform_tolerance_))
  {
    // 如果转换成功，将转换后的机器人位姿存储在 robot_pose 中，否则，
    // 会抛出异常 dwb_core::PlannerTFException，表示无法将机器人位姿转换到全局路径的坐标系中
    throw dwb_core::
          PlannerTFException("Unable to transform robot pose into global plan's frame");
  }

  // we'll discard points on the plan that are outside the local costmap
  // 调用 costmap_ros_->getCostmap() 获取到当前的 costmap 对象，表示机器人所在的成本地图
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  // dist_threshold 计算的是一个距离阈值，其值为机器人所在的成本地图中较大的一边的网格数目乘以
  // 每个网格的分辨率的一半。这个值用来限制路径中的点到成本地图的边界的最大距离，以便进行修剪。
  // 这里假设成本地图是一个正方形，通过计算较大的一边的网格数目乘以分辨率的一半来获得一个合适的距离阈值
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;

  // If prune_plan is enabled (it is by default) then we want to restrict the
  // plan to distances within that range as well.
  // 指定修剪的距离阈值
  double prune_dist = prune_distance_;

  // Set the maximum distance we'll include points before getting to the part
  // of the path where the robot is located (the start of the plan). Basically,
  // these are the points the robot has already passed.
  double transform_start_threshold;
  // 如果启用了路径修剪，则 transform_start_threshold 将被设置为 dist_threshold 和 prune_dist 中的较小值
  if (prune_plan_) {
    transform_start_threshold = std::min(dist_threshold, prune_dist);
  } else {  // 将直接使用 dist_threshold
    transform_start_threshold = dist_threshold;
  }

  // Set the maximum distance we'll include points after the part of the plan
  // near the robot (the end of the plan). This determines the amount of the
  // plan passed on to the critics
  // 根据是否启用了 shorten_transformed_plan_，选择合适的终止位置的距离阈值 transform_end_threshold
  double transform_end_threshold;
  double forward_prune_dist = forward_prune_distance_;
  // 如果启用了 shorten_transformed_plan_，则将其设置为 dist_threshold 和 forward_prune_dist 中的较小值
  if (shorten_transformed_plan_) {
    transform_end_threshold = std::min(dist_threshold, forward_prune_dist);
  } else {    // 否则，将直接使用 dist_threshold
    transform_end_threshold = dist_threshold;
  }

  // Find the first pose in the global plan that's further than prune distance
  // from the robot using integrated distance
  // 根据指定的 prune_dist，使用 nav2_util::geometry_utils::first_after_integrated_distance 
  // 函数来找到全局路径中第一个距离机器人大于 prune_dist 的点，即需要进行修剪的起始点。
  // 这个函数的目的是找到一个在指定距离后的路径点
  auto prune_point = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), prune_dist);

  // Find the first pose in the plan (upto prune_point) that's less than transform_start_threshold
  // from the robot.
  // 通过迭代全局路径，找到一个在距离机器人起始点的距离小于 transform_start_threshold 的位置，
  // 作为修剪变换的起始位置
  auto transformation_begin = std::find_if(
    begin(global_plan_.poses), prune_point,
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose.pose, global_plan_pose) < transform_start_threshold;
    });

  // Find the first pose in the end of the plan that's further than transform_end_threshold
  // from the robot using integrated distance
  // 再次使用 std::find_if 函数，从 transformation_begin（修剪变换的起始位置）开始迭代全局路径，
  // 找到一个距离机器人终止点的距离大于 transform_end_threshold 的位置，作为修剪变换的终止位置
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) {
      return euclidean_distance(pose, robot_pose.pose) > transform_end_threshold;
    });

  // Transform the near part of the global plan into the robot's frame of reference.
  // 创建一个空的 transformed_plan 变量，用于存储经过变换后的路径。设置该变量的头部信息，
  // 包括 frame_id（全局坐标系的标识）和时间戳
  nav_2d_msgs::msg::Path2D transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Helper function for the transform below. Converts a pose2D from global
  // frame to local
  // 使用 Lambda 函数 transformGlobalPoseToLocal 来定义全局坐标系下的路径点如何转换为
  // 本地坐标系下的路径点。在 Lambda 函数内部，首先将全局坐标系的路径点构造成一个带有时间戳的
  // Pose2DStamped 消息，然后使用 nav_2d_utils::transformPose 函数将路径点
  // 从全局坐标系转换到本地坐标系，最终返回转换后的本地坐标系路径点
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      nav_2d_msgs::msg::Pose2DStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;   // 将 stamped_pose 的头部信息中的 frame_id 设置为全局路径的坐标系标识
      stamped_pose.pose = global_plan_pose;   // 代表全局坐标系下的路径点
      nav_2d_utils::transformPose(
        tf_, transformed_plan.header.frame_id,  //  函数将 stamped_pose 进行坐标变换，将其从全局坐标系转换到本地坐标系。传递的参数包括 tf_（变换工具）
        stamped_pose, transformed_pose, transform_tolerance_);  // 带有时间戳的二维位姿信息
      return transformed_pose.pose; // 将变换后的位姿信息从 transformed_pose 返回，这将是本地坐标系下的路径
    };
  // 使用 std::transform 函数，将路径中从 transformation_begin 到 transformation_end 范围内的路径点，
  // 依次通过 transformGlobalPoseToLocal 函数进行坐标变换，然后将结果添加到 transformed_plan.poses 中
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration.
  // 如果启用了 prune_plan_，则删除 global_plan_ 中在 transformation_begin 之前的路径点，然后发布修剪后的全局路径
  if (prune_plan_) {
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    pub_->publishGlobalPlan(global_plan_);
  }
  // 检查经过变换和修剪后的路径是否为空，如果为空，则抛出一个 nav2_core::PlannerException 异常
  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }
  return transformed_plan;
}

}  // namespace dwb_core

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  dwb_core::DWBLocalPlanner,
  nav2_core::Controller)
