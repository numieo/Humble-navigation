// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Florian Gramss
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_behavior_tree/behavior_tree_engine.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/utils/shared_library.h"

namespace nav2_behavior_tree
{

BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries)
{
  // 创建一个 BT::SharedLibrary 类的实例 loader，用于加载插件库
  BT::SharedLibrary loader;
  // 遍历 plugin_libraries 中的每个插件库名称
  for (const auto & p : plugin_libraries) {
    // 通过 factory_ 对象，从指定的插件库加载节点并注册到工厂中
    factory_.registerFromPlugin(loader.getOSName(p));
  }
}

// 在 nav2_behavior_tree 命名空间内定义的自定义类型，表示行为树引擎的状态
BtStatus
BehaviorTreeEngine::run(    // BehaviorTreeEngine 类的 run 方法，用于运行给定的行为树
  BT::Tree * tree,          // 指向要执行的行为树的指针
  std::function<void()> onLoop,    // 一个回调函数，表示每个循环迭代中要执行的操作
  std::function<bool()> cancelRequested,  // 一个回调函数，用于检查是否取消了行为树的执行
  std::chrono::milliseconds loopTimeout)  // 循环的超时时间，以毫秒为单位
{
  rclcpp::WallRate loopRate(loopTimeout); // 创建一个用于控制循环频率的 rclcpp::WallRate 对象，其参数是循环超时时间
  BT::NodeStatus result = BT::NodeStatus::RUNNING; // 初始化 result 变量为 RUNNING，表示行为树的初始状态为运行中

  // Loop until something happens with ROS or the node completes
  try {
    // 在循环中，只要 ROS 2 的节点正常运行且行为树的根节点返回状态为运行中，就继续执行循环
    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
      if (cancelRequested()) {    // 如果 cancelRequested 回调函数返回 true
        tree->rootNode()->halt(); // 通过调用根节点的 halt 方法，停止执行整个行为树
        return BtStatus::CANCELED;  // 表示行为树的执行已被取消
      }

      result = tree->tickRoot();    // 调用行为树的 tickRoot 方法，执行根节点的行为

      onLoop();     // 调用在每个循环迭代中执行的用户定义的回调函数

      loopRate.sleep();   // 控制循环的频率，使每次循环迭代之间的时间间隔达到指定的超时时间
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BehaviorTreeEngine"),
      "Behavior tree threw exception: %s. Exiting with failure.", ex.what());
    return BtStatus::FAILED;
  }
  // 检查根节点的返回状态。如果根节点的状态为成功，则返回 BtStatus::SUCCEEDED 表示行为树的执行成功，否则返回 BtStatus::FAILED 表示行为树的执行失败
  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

/**
 * @brief 从给定的 XML 字符串创建行为树
 * 
 * @param xml_string 包含行为树描述的 XML 字符串
 * @param blackboard 指向黑板（blackboard）的智能指针，用于共享数据
 * @return BT::Tree 表示创建的行为树
 */
BT::Tree
BehaviorTreeEngine::createTreeFromText(
  const std::string & xml_string,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromText(xml_string, blackboard);
}

/**
 * @brief createTreeFromFile 方法用于从指定的文件路径创建行为树
 * 
 * @param file_path 行为树描述文件的路径
 * @param blackboard 一个指向黑板（blackboard）的智能指针，用于共享数据
 * @return BT::Tree 表示创建的行为树
 */
BT::Tree
BehaviorTreeEngine::createTreeFromFile(
  const std::string & file_path,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromFile(file_path, blackboard);
}

// In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
void
BehaviorTreeEngine::haltAllActions(BT::TreeNode * root_node)
{
  if (!root_node) {
    return;
  }

  // this halt signal should propagate through the entire tree.
  // 停止根节点，使终止信号向下传播到整个树
  root_node->halt();

  // but, just in case...
  // 但是，为了确保，还要继续处理...

  // 使用 visitor 函数来访问行为树的每个节点。如果节点的状态为 RUNNING，则调用 halt() 方法停止该节点
  auto visitor = [](BT::TreeNode * node) {
      if (node->status() == BT::NodeStatus::RUNNING) {
        node->halt();
      }
    };
  // 通过调用 BT::applyRecursiveVisitor(root_node, visitor) 来应用 visitor 函数到整个行为树，确保所有正在运行的节点都被停止
  BT::applyRecursiveVisitor(root_node, visitor);
}

}  // namespace nav2_behavior_tree
