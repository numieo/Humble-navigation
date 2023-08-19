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

#include "dwb_core/illegal_trajectory_tracker.hpp"
#include <map>
#include <utility>
#include <string>
#include <sstream>

namespace dwb_core
{
// 将一个非法轨迹异常添加到跟踪器中。它接受一个 IllegalTrajectoryException 
// 类型的异常对象作为参数。函数将该异常的批评者名称（getCriticName()）和
// 异常信息（what()）结合成一个键，并将该键对应的计数增加。同时，非法轨迹的总数也会增加
void IllegalTrajectoryTracker::addIllegalTrajectory(
  const dwb_core::IllegalTrajectoryException & e)
{
  counts_[std::make_pair(e.getCriticName(), e.what())]++;
  illegal_count_++;
}
// 将一个合法轨迹添加到跟踪器中。每当发现一个合法轨迹时，调用这个函数来增加合法轨迹的计数
void IllegalTrajectoryTracker::addLegalTrajectory()
{
  legal_count_++;
}

// 计算不同批评者和异常信息组合的非法轨迹比例。它返回一个 std::map，
// 键是一个包含批评者名称和异常信息的 std::pair，值是相应的非法轨迹比例。
// 比例计算是通过将每个组合的非法轨迹计数除以合法轨迹和非法轨迹的总和得到的
std::map<std::pair<std::string, std::string>,
  double> IllegalTrajectoryTracker::getPercentages() const
{
  std::map<std::pair<std::string, std::string>, double> percents;
  double denominator = static_cast<double>(legal_count_ + illegal_count_);
  for (auto const & x : counts_) {
    percents[x.first] = static_cast<double>(x.second) / denominator;
  }
  return percents;
}

// 生成一个关于非法轨迹统计信息的消息字符串。这个函数根据合法轨迹计数和非法轨迹计数，
// 生成一个包含统计信息的消息，以字符串形式返回
std::string IllegalTrajectoryTracker::getMessage() const
{
  std::ostringstream msg;
  if (legal_count_ == 0) {
    // 如果 legal_count_（合法轨迹计数）为0，表示没有找到合法轨迹，函数会生成一个消息字符串，说明找到了多少条非法轨迹
    msg << "No valid trajectories out of " << illegal_count_ << "! ";
  } else {
    // 生成一个消息字符串，说明找到了多少条合法轨迹，以及合法轨迹占总轨迹的百分比
    unsigned int total = legal_count_ + illegal_count_;
    msg << legal_count_ << " valid trajectories found (";
    msg << static_cast<double>(100 * legal_count_) / static_cast<double>(total);
    msg << "% of " << total << "). ";
  }
  return msg.str();
}

}  // namespace dwb_core
