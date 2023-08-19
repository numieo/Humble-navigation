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

#include "nav_2d_utils/path_ops.hpp"
#include <cmath>

using std::sqrt;

namespace nav_2d_utils
{
nav_2d_msgs::msg::Path2D adjustPlanResolution(
  const nav_2d_msgs::msg::Path2D & global_plan_in,
  double resolution)
{
  // 在调整分辨率后的路径规划消息，用于存储调整后的路径
  nav_2d_msgs::msg::Path2D global_plan_out;
  // 如果输入的路径规划消息中没有任何点，直接返回空的调整后路径，不进行进一步的处理
  if (global_plan_in.poses.size() == 0) {
    return global_plan_out;
  }

  // 将输入路径规划消息的第一个点作为起始点
  geometry_msgs::msg::Pose2D last = global_plan_in.poses[0];
  // 将起始点添加到调整后的路径中
  global_plan_out.poses.push_back(last);

  // we can take "holes" in the plan smaller than 2 grid cells (squared = 4)
  // 计算分辨率的平方乘以 4，作为最小的平方分辨率，用于检测路径规划中的“空洞”
  double min_sq_resolution = resolution * resolution * 4.0;

  // 从路径规划消息的第二个点开始遍历，以便与前一个点进行比较并进行分辨率调整
  for (unsigned int i = 1; i < global_plan_in.poses.size(); ++i) {
    // 获取当前循环中的路径规划点
    geometry_msgs::msg::Pose2D loop = global_plan_in.poses[i];
    // 计算当前点与上一个点之间的平方距离
    double sq_dist = (loop.x - last.x) * (loop.x - last.x) + (loop.y - last.y) * (loop.y - last.y);
    // 如果平方距离大于最小平方分辨率，表示距离太大，需要插入额外的点以调整分辨率
    if (sq_dist > min_sq_resolution) {
      // add points in-between
      // 计算距离差，用于确定插入多少额外的点以满足分辨率
      double diff = sqrt(sq_dist) - sqrt(min_sq_resolution);
      // 计算需要插入的额外点的数量，以满足分辨率要求
      int steps = static_cast<int>(diff / resolution) - 1;
      double steps_double = static_cast<double>(steps);
      // 计算插入的每个点在 x、y 和角度方向上的增量
      double delta_x = (loop.x - last.x) / steps_double;
      double delta_y = (loop.y - last.y) / steps_double;
      double delta_t = (loop.theta - last.theta) / steps_double;
      // 在两个点之间插入额外的点，以满足分辨率要求。计算每个插入点的位置和角度，将其添加到调整后的路径中
      for (int j = 1; j < steps; ++j) {
        geometry_msgs::msg::Pose2D pose;
        pose.x = last.x + j * delta_x;
        pose.y = last.y + j * delta_y;
        pose.theta = last.theta + j * delta_t;
        global_plan_out.poses.push_back(pose);
      }
    }
    // 将当前路径规划点添加到调整后的路径中
    global_plan_out.poses.push_back(global_plan_in.poses[i]);
    // 更新上一个点的坐标为当前点的坐标
    last.x = loop.x;
    last.y = loop.y;
  }
  // 循环结束后，将调整后的路径 global_plan_out 返回
  return global_plan_out;
}
}  // namespace nav_2d_utils
