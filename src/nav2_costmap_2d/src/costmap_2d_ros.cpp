/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <utility>

#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/execution_timer.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/robot_utils.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{
Costmap2DROS::Costmap2DROS(const std::string & name)
: Costmap2DROS(name, "/", name) {}

Costmap2DROS::Costmap2DROS(
  const std::string & name,
  const std::string & parent_namespace,
  const std::string & local_namespace)
: nav2_util::LifecycleNode(name, "",
    // NodeOption arguments take precedence over the ones provided on the command line
    // use this to make sure the node is placed on the provided namespace
    // TODO(orduno) Pass a sub-node instead of creating a new node for better handling
    //              of the namespaces
    rclcpp::NodeOptions().arguments({
    "--ros-args", "-r", std::string("__ns:=") +
    nav2_util::add_namespaces(parent_namespace, local_namespace),
    "--ros-args", "-r", name + ":" + std::string("__node:=") + name
  })),
  name_(name),
  parent_namespace_(parent_namespace),
  default_plugins_{"static_layer", "obstacle_layer", "inflation_layer"},
  default_types_{
    "nav2_costmap_2d::StaticLayer",
    "nav2_costmap_2d::ObstacleLayer",
    "nav2_costmap_2d::InflationLayer"}
{
  RCLCPP_INFO(get_logger(), "Creating Costmap");

  std::vector<std::string> clearable_layers{"obstacle_layer", "voxel_layer", "range_layer"};

  declare_parameter("always_send_full_costmap", rclcpp::ParameterValue(false));
  declare_parameter("footprint_padding", rclcpp::ParameterValue(0.01f));
  declare_parameter("footprint", rclcpp::ParameterValue(std::string("[]")));
  declare_parameter("global_frame", rclcpp::ParameterValue(std::string("map")));
  declare_parameter("height", rclcpp::ParameterValue(5));
  declare_parameter("width", rclcpp::ParameterValue(5));
  declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
  declare_parameter(
    "map_topic", rclcpp::ParameterValue(
      (parent_namespace_ == "/" ? "/" : parent_namespace_ + "/") + std::string("map")));
  declare_parameter("observation_sources", rclcpp::ParameterValue(std::string("")));
  declare_parameter("origin_x", rclcpp::ParameterValue(0.0));
  declare_parameter("origin_y", rclcpp::ParameterValue(0.0));
  declare_parameter("plugins", rclcpp::ParameterValue(default_plugins_));
  declare_parameter("filters", rclcpp::ParameterValue(std::vector<std::string>()));
  declare_parameter("publish_frequency", rclcpp::ParameterValue(1.0));
  declare_parameter("resolution", rclcpp::ParameterValue(0.1));
  declare_parameter("robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter("robot_radius", rclcpp::ParameterValue(0.1));
  declare_parameter("rolling_window", rclcpp::ParameterValue(false));
  declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
  declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
  declare_parameter("unknown_cost_value", rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
  declare_parameter("update_frequency", rclcpp::ParameterValue(5.0));
  declare_parameter("use_maximum", rclcpp::ParameterValue(false));
  declare_parameter("clearable_layers", rclcpp::ParameterValue(clearable_layers));
}

Costmap2DROS::~Costmap2DROS()
{
}

// 生命周期节点配置阶段的回调函数
nav2_util::CallbackReturn
Costmap2DROS::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  getParameters();  // 获取参数配置
  // 创建一个回调组（callback group）来管理回调函数的执行
  callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);

  // Create the costmap itself
  // 创建 costmap 实例
  layered_costmap_ = std::make_unique<LayeredCostmap>(
    global_frame_, rolling_window_, track_unknown_space_);
  // 检查 layered_costmap_ 是否锁定大小，如果没有锁定，则通过 resizeMap 函数设置地图的大小和分辨率
  if (!layered_costmap_->isSizeLocked()) {
    layered_costmap_->resizeMap(
      (unsigned int)(map_width_meters_ / resolution_),
      (unsigned int)(map_height_meters_ / resolution_), resolution_, origin_x_, origin_y_);
  }

  // Create the transform-related objects
  // 创建 tf2_ros::Buffer 对象，用于管理变换关系。将该对象设置为共享的智能指针，传递给 tf2_ros::CreateTimerROS 构造函数作为定时器接口
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface(),
    callback_group_);
  tf_buffer_->setCreateTimerInterface(timer_interface);
  // 用于监听变换关系的更新
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Then load and add the plug-ins to the costmap
  // 加载和添加插件到代价地图（Costmap2D）中。通过遍历 plugin_names_，创建并初始化每个插件，并将它们添加到代价地图的层中
  for (unsigned int i = 0; i < plugin_names_.size(); ++i) {
    RCLCPP_INFO(get_logger(), "Using plugin \"%s\"", plugin_names_[i].c_str());
    // // 创建插件实例
    std::shared_ptr<Layer> plugin = plugin_loader_.createSharedInstance(plugin_types_[i]);

    // lock the costmap because no update is allowed until the plugin is initialized
    // 锁定代价地图，因为在插件初始化之前不允许更新
    std::unique_lock<Costmap2D::mutex_t> lock(*(layered_costmap_->getCostmap()->getMutex()));
    // // 将插件添加到代价地图
    layered_costmap_->addPlugin(plugin);

    // TODO(mjeronimo): instead of get(), use a shared ptr
    // // 初始化插件
    plugin->initialize(
      layered_costmap_.get(), plugin_names_[i], tf_buffer_.get(),
      shared_from_this(), callback_group_);
    // 显式释放代价地图的锁，使其他线程能够继续访问和更新代价地图
    lock.unlock();

    RCLCPP_INFO(get_logger(), "Initialized plugin \"%s\"", plugin_names_[i].c_str());
  }
  // and costmap filters as well
  // 遍历每个滤波器名称
  for (unsigned int i = 0; i < filter_names_.size(); ++i) {
    // 打印日志，说明正在使用哪个代价地图滤波器
    RCLCPP_INFO(get_logger(), "Using costmap filter \"%s\"", filter_names_[i].c_str());
    // 使用插件加载器(plugin_loader_)创建一个共享实例(std::shared_ptr)，对应于当前迭代的滤波器类型(filter_types_[i])
    std::shared_ptr<Layer> filter = plugin_loader_.createSharedInstance(filter_types_[i]);

    // lock the costmap because no update is allowed until the filter is initialized
    // 使用互斥锁将代价地图进行加锁，以确保在滤波器初始化过程中不会进行更新操作
    std::unique_lock<Costmap2D::mutex_t> lock(*(layered_costmap_->getCostmap()->getMutex()));
    // 将当前创建的滤波器实例添加到层叠代价地图
    layered_costmap_->addFilter(filter);
    // 传递层叠代价地图、滤波器名称、TF2的缓冲区、以及当前Costmap2DROS实例
    filter->initialize(
      layered_costmap_.get(), filter_names_[i], tf_buffer_.get(),
      shared_from_this(), callback_group_);
    // 解锁代价地图，允许其他线程进行访问和更新
    lock.unlock();
    // 说明已经初始化了哪个代价地图滤波器
    RCLCPP_INFO(get_logger(), "Initialized costmap filter \"%s\"", filter_names_[i].c_str());
  }

  // Create the publishers and subscribers
  footprint_sub_ = create_subscription<geometry_msgs::msg::Polygon>(
    "footprint",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&Costmap2DROS::setRobotFootprintPolygon, this, std::placeholders::_1));

  footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
    "published_footprint", rclcpp::SystemDefaultsQoS());
  // 使用 Costmap2DPublisher 类创建一个名为 costmap_publisher_ 的唯一指针，
  // 用于发布代价地图相关信息。这里会传递当前的 Costmap2DROS 实例、代价地图、
  // 全局坐标系名称、代价地图的发布名称以及一个标志 (always_send_full_costmap_)
  costmap_publisher_ = std::make_unique<Costmap2DPublisher>(
    shared_from_this(),
    layered_costmap_->getCostmap(), global_frame_,
    "costmap", always_send_full_costmap_);

  // Set the footprint
  // 根据初始化参数来设置机器人的足迹（footprint）。如果 use_radius_ 为 true，
  // 则通过 makeFootprintFromRadius 方法创建一个基于机器人半径的足迹；否则，
  // 通过 makeFootprintFromString 方法根据给定的字符串来创建足迹，并调用 setRobotFootprint 方法来设置机器人的足迹
  if (use_radius_) {
    setRobotFootprint(makeFootprintFromRadius(robot_radius_));
  } else {
    std::vector<geometry_msgs::msg::Point> new_footprint;
    makeFootprintFromString(footprint_, new_footprint);
    setRobotFootprint(new_footprint);
  }

  // Add cleaning service
  // 清除代价地图的服务
  clear_costmap_service_ = std::make_unique<ClearCostmapService>(shared_from_this(), *this);
  // 创建一个单线程执行器 (executor_)，这个执行器会用来执行代价地图的相关操作
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  // 获取节点的基本接口
  executor_->add_callback_group(callback_group_, get_node_base_interface());
  // 该实例会将执行器放在一个独立的线程中运行，从而避免阻塞主线程
  executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);
  // 表示 on_configure 方法执行成功，代价地图的配置和初始化已经完成
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Costmap2DROS::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  // 表示代价地图正在被激活
  RCLCPP_INFO(get_logger(), "Activating");
  // 激活与代价地图相关的 Costmap2DPublisher
  costmap_publisher_->on_activate();
  // 激活发布机器人足迹多边形的话题
  footprint_pub_->on_activate();

  // First, make sure that the transform between the robot base frame
  // and the global frame is available

  std::string tf_error;

  RCLCPP_INFO(get_logger(), "Checking transform");
  // 检查机器人基准框架 (robot_base_frame_) 到全局框架 (global_frame_) 的变换是否可用。
  // 使用 tf_buffer_->canTransform 函数检查变换是否可用，如果不可用，会在一定时间内
  // 等待直到变换可用为止。循环中的日志信息会打印出等待过程中的状态
  rclcpp::Rate r(2);
  while (rclcpp::ok() &&
    !tf_buffer_->canTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero, &tf_error))
  {
    RCLCPP_INFO(
      get_logger(), "Timed out waiting for transform from %s to %s"
      " to become available, tf error: %s",
      robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());

    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation
    tf_error.clear();
    r.sleep();
  }

  // Create a thread to handle updating the map
  stopped_ = true;  // to active plugins
  stop_updates_ = false;  // 确保代价地图更新不会在这个阶段停止
  map_update_thread_shutdown_ = false;  // 确保地图更新线程不会在这个阶段关闭
  // 调用 mapUpdateLoop 方法来执行地图更新循环。这个循环会根据指定的频率 (map_update_frequency_) 更新代价地图
  map_update_thread_ = std::make_unique<std::thread>(
    std::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency_));

  start();

  // Add callback for dynamic parameters
  // 用于处理动态参数的更新。这个回调函数可能用于在代价地图激活后监控和响应动态参数的变化
  dyn_params_handler = this->add_on_set_parameters_callback(
    std::bind(&Costmap2DROS::dynamicParametersCallback, this, _1));
  // 表示代价地图的 on_activate 方法成功执行完成
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Costmap2DROS::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  dyn_params_handler.reset();           // 停止对动态参数的更新监听
  costmap_publisher_->on_deactivate();  // 停止代价地图的发布器，防止继续发布代价地图数据
  footprint_pub_->on_deactivate();      // 停止机器人足迹的发布

  stop(); // 停止代价地图的更新

  // Map thread stuff
  map_update_thread_shutdown_ = true; // 通知地图更新线程停止
  map_update_thread_->join();         // 等待地图更新线程的结束
  // 表示代价地图的 on_deactivate 方法成功执行完成
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Costmap2DROS::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  layered_costmap_.reset();       // 释放代价地图的内存

  tf_listener_.reset();           // 释放用于监听 TF 变换的对象的内存。
  tf_buffer_.reset();             // 释放用于存储 TF 变换缓冲区的对象的内存。

  footprint_sub_.reset();         // 释放机器人足迹订阅器的内存
  footprint_pub_.reset();         // 释放机器人足迹发布器的内存

  costmap_publisher_.reset();     // 释放代价地图发布器的内存
  clear_costmap_service_.reset(); // 释放清除代价地图服务对象的内存

  executor_thread_.reset();       // 释放执行器线程的内存
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Costmap2DROS::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
Costmap2DROS::getParameters()
{
  RCLCPP_DEBUG(get_logger(), " getParameters");

  // Get all of the required parameters
  get_parameter("always_send_full_costmap", always_send_full_costmap_);
  get_parameter("footprint", footprint_);
  get_parameter("footprint_padding", footprint_padding_);
  get_parameter("global_frame", global_frame_);
  get_parameter("height", map_height_meters_);
  get_parameter("origin_x", origin_x_);
  get_parameter("origin_y", origin_y_);
  get_parameter("publish_frequency", map_publish_frequency_);
  get_parameter("resolution", resolution_);
  get_parameter("robot_base_frame", robot_base_frame_);
  get_parameter("robot_radius", robot_radius_);
  get_parameter("rolling_window", rolling_window_);
  get_parameter("track_unknown_space", track_unknown_space_);
  get_parameter("transform_tolerance", transform_tolerance_);
  get_parameter("update_frequency", map_update_frequency_);
  get_parameter("width", map_width_meters_);
  get_parameter("plugins", plugin_names_);
  get_parameter("filters", filter_names_);

  auto node = shared_from_this();

  if (plugin_names_ == default_plugins_) {
    for (size_t i = 0; i < default_plugins_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_plugins_[i] + ".plugin", rclcpp::ParameterValue(default_types_[i]));
    }
  }
  plugin_types_.resize(plugin_names_.size());
  filter_types_.resize(filter_names_.size());

  // 1. All plugins must have 'plugin' param defined in their namespace to define the plugin type
  for (size_t i = 0; i < plugin_names_.size(); ++i) {
    plugin_types_[i] = nav2_util::get_plugin_type_param(node, plugin_names_[i]);
  }
  for (size_t i = 0; i < filter_names_.size(); ++i) {
    filter_types_[i] = nav2_util::get_plugin_type_param(node, filter_names_[i]);
  }

  // 2. The map publish frequency cannot be 0 (to avoid a divde-by-zero)
  if (map_publish_frequency_ > 0) {
    publish_cycle_ = rclcpp::Duration::from_seconds(1 / map_publish_frequency_);
  } else {
    publish_cycle_ = rclcpp::Duration(-1s);
  }

  // 3. If the footprint has been specified, it must be in the correct format
  use_radius_ = true;

  if (footprint_ != "" && footprint_ != "[]") {
    // Footprint parameter has been specified, try to convert it
    std::vector<geometry_msgs::msg::Point> new_footprint;
    if (makeFootprintFromString(footprint_, new_footprint)) {
      // The specified footprint is valid, so we'll use that instead of the radius
      use_radius_ = false;
    } else {
      // Footprint provided but invalid, so stay with the radius
      RCLCPP_ERROR(
        get_logger(), "The footprint parameter is invalid: \"%s\", using radius (%lf) instead",
        footprint_.c_str(), robot_radius_);
    }
  }
}

void
Costmap2DROS::setRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points)
{
  // 未填充和已填充的机器人足迹
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  // 调用 padFootprint 方法对 padded_footprint_ 进行填充，填充的程度由 footprint_padding_ 参数指定
  padFootprint(padded_footprint_, footprint_padding_);
  // 将填充后的足迹设置到代价地图中
  layered_costmap_->setFootprint(padded_footprint_);
}

// 足迹多边形指针作为参数
void
Costmap2DROS::setRobotFootprintPolygon(
  const geometry_msgs::msg::Polygon::SharedPtr footprint)
{
  // 调用 toPointVector 方法将足迹多边形转换为 geometry_msgs::msg::Point 的点集合
  // 将转换后的点集合作为参数设置机器人足迹
  setRobotFootprint(toPointVector(footprint));
}

void
Costmap2DROS::getOrientedFootprint(std::vector<geometry_msgs::msg::Point> & oriented_footprint)
{
  // oriented_footprint，用于存储获取的旋转足迹
  // global_pose，用于存储机器人的全局位姿
  geometry_msgs::msg::PoseStamped global_pose;
  // 调用 getRobotPose 方法获取机器人的全局位姿，并将位姿信息存储到 global_pose 中
  if (!getRobotPose(global_pose)) {
    return;
  }
  // 提取机器人的朝向角度 yaw，使用 tf2::getYaw 函数从姿态四元数中获取机器人的朝向角度
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  // 将机器人的当前位置、朝向角度、已填充的足迹以及空的 oriented_footprint 作为参数，将机器人的旋转足迹计算并存储到 oriented_footprint 中
  transformFootprint(
    global_pose.pose.position.x, global_pose.pose.position.y, yaw,
    padded_footprint_, oriented_footprint);
}

void
Costmap2DROS::mapUpdateLoop(double frequency)
{
  // 打印出循环的频率
  RCLCPP_DEBUG(get_logger(), "mapUpdateLoop frequency: %lf", frequency);

  // the user might not want to run the loop every cycle
  // 如果频率为0.0，表示用户不希望在每个循环周期中执行此循环，因此直接返回，结束方法
  // 如果频率不为0.0，表示需要在循环中执行地图更新
  if (frequency == 0.0) {
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Entering loop");
  // 使用给定的频率来设置循环的频率
  rclcpp::WallRate r(frequency);    // 200ms by default

  // 该循环会在ROS节点仍在运行且map_update_thread_shutdown_标志为false时继续执行。
  // map_update_thread_shutdown_标志用于控制循环是否终止
  while (rclcpp::ok() && !map_update_thread_shutdown_) {
    nav2_util::ExecutionTimer timer;

    // Execute after start() will complete plugins activation
    // 如果 stopped_ 标志为 false，则进入这个条件块。这是因为只有在地图更新被激活时才需要执行地图更新
    if (!stopped_) {
      // Measure the execution time of the updateMap method
      // 启动计时器，用于测量地图更新操作的执行时间
      timer.start();
      // 调用 updateMap 方法，执行地图的实际更新操作
      updateMap();
      // 停止计时器，计算地图更新操作的执行时间
      timer.end();

      RCLCPP_DEBUG(get_logger(), "Map update time: %.9f", timer.elapsed_time_in_seconds());
      // 如果定义了 publish_cycle_（地图发布周期），并且分层成本图已初始化，那么在这个条件块中进行地图发布操作
      if (publish_cycle_ > rclcpp::Duration(0s) && layered_costmap_->isInitialized()) {
        unsigned int x0, y0, xn, yn;
        // 获取地图的边界坐标并更新地图发布器的边界
        layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
        costmap_publisher_->updateBounds(x0, xn, y0, yn);

        auto current_time = now();
        // 获取当前时间并检查是否应该发布地图。如果满足发布条件，就执行地图发布操作，并记录最后一次发布的时间
        if ((last_publish_ + publish_cycle_ < current_time) ||  // publish_cycle_ is due
          (current_time < last_publish_))      // time has moved backwards, probably due to a switch to sim_time // NOLINT
        {
          RCLCPP_DEBUG(get_logger(), "Publish costmap at %s", name_.c_str());
          costmap_publisher_->publishCostmap();
          last_publish_ = current_time;
        }
      }
    }

    // Make sure to sleep for the remainder of our cycle time
    // 在循环的每次迭代结束时，通过调用 r.sleep() 方法，确保在循环周期的剩余时间内进行休眠。这样可以控制循环的频率
    r.sleep();

#if 0
    // TODO(bpwilcox): find ROS2 equivalent or port for r.cycletime()
    // 警告地图更新循环的执行时间是否符合预期频率的部分
    if (r.period() > tf2::durationFromSec(1 / frequency)) {
      RCLCPP_WARN(
        get_logger(),
        "Costmap2DROS: Map update loop missed its desired rate of %.4fHz... "
        "the loop actually took %.4f seconds", frequency, r.period());
    }
#endif
  }
}

void
Costmap2DROS::updateMap()
{
  RCLCPP_DEBUG(get_logger(), "Updating map...");

  // 检查是否允许地图更新操作。stop_updates_ 标志为 false 时才会进行地图更新
  if (!stop_updates_) {
    // get global pose
    // 通过调用 getRobotPose 方法获取机器人的全局姿态，包括位置和方向
    geometry_msgs::msg::PoseStamped pose;
    if (getRobotPose(pose)) {
      const double & x = pose.pose.position.x;
      const double & y = pose.pose.position.y;
      const double yaw = tf2::getYaw(pose.pose.orientation);
      // 如果成功获取机器人的全局姿态，将获取到的位置和方向传递给 layered_costmap_->updateMap 
      // 方法执行地图的更新。这可能会根据机器人当前的位置和方向来更新地图上的成本值
      layered_costmap_->updateMap(x, y, yaw);

      auto footprint = std::make_unique<geometry_msgs::msg::PolygonStamped>();
      footprint->header = pose.header;
      transformFootprint(x, y, yaw, padded_footprint_, *footprint);

      RCLCPP_DEBUG(get_logger(), "Publishing footprint");
      // 根据机器人的位置和方向，将机器人足迹进行变换，并将变换后的足迹通过 footprint_pub_ 发布出去
      footprint_pub_->publish(std::move(footprint));
      initialized_ = true;  // 表示地图已经初始化
    }
  }
}

void
Costmap2DROS::start()
{
  RCLCPP_INFO(get_logger(), "start");
  // 获取当前地图中的插件和过滤器，并通过迭代它们来逐个激活。这些插件和过滤器在地图更新过程中负责计算地图的成本值
  std::vector<std::shared_ptr<Layer>> * plugins = layered_costmap_->getPlugins();
  std::vector<std::shared_ptr<Layer>> * filters = layered_costmap_->getFilters();

  // check if we're stopped or just paused
  // 如果地图之前是停止状态（stopped_ 为 true），说明需要重新订阅话题并激活插件和过滤器
  if (stopped_) {
    // if we're stopped we need to re-subscribe to topics
    for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins->begin();
      plugin != plugins->end();
      ++plugin)
    {
      (*plugin)->activate();
    }
    for (std::vector<std::shared_ptr<Layer>>::iterator filter = filters->begin();
      filter != filters->end();
      ++filter)
    {
      (*filter)->activate();
    }
    stopped_ = false;
  }
  // stopped_ 标志被重置，表示地图已经开始更新。stop_updates_ 标志被重置，表示允许地图更新操作
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  rclcpp::Rate r(20.0);
  // 确保地图已经完成初始化并执行了至少一次更新循环。这个等待是为了确保地图已经准备好进行实际的更新操作
  while (rclcpp::ok() && !initialized_) {
    RCLCPP_DEBUG(get_logger(), "Sleeping, waiting for initialized_");
    r.sleep();
  }
}

void
Costmap2DROS::stop()
{
  stop_updates_ = true; // 表示停止地图的更新。这个标志会阻止地图继续执行更新循环
  // 获取当前地图中的插件和过滤器，以便之后进行迭代操作
  std::vector<std::shared_ptr<Layer>> * plugins = layered_costmap_->getPlugins();
  std::vector<std::shared_ptr<Layer>> * filters = layered_costmap_->getFilters();
  // unsubscribe from topics
  // 通过迭代插件和过滤器，逐个调用它们的 deactivate 方法，将它们停用。这样地图将不再进行成本值的计算
  for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins->begin();
    plugin != plugins->end(); ++plugin)
  {
    (*plugin)->deactivate();
  }
  for (std::vector<std::shared_ptr<Layer>>::iterator filter = filters->begin();
    filter != filters->end(); ++filter)
  {
    (*filter)->deactivate();
  }
  initialized_ = false; // 将 initialized_ 标志设置为 false，表示地图不再处于初始化状态。
  stopped_ = true;  // 将 stopped_ 标志设置为 true，表示地图已经停止
}

void
Costmap2DROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}

void
Costmap2DROS::resume()
{
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  rclcpp::Rate r(100.0);
  while (!initialized_) {
    r.sleep();
  }
}

void
Costmap2DROS::resetLayers()
{
  Costmap2D * top = layered_costmap_->getCostmap();
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());

  // Reset each of the plugins
  std::vector<std::shared_ptr<Layer>> * plugins = layered_costmap_->getPlugins();
  std::vector<std::shared_ptr<Layer>> * filters = layered_costmap_->getFilters();
  for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins->begin();
    plugin != plugins->end(); ++plugin)
  {
    (*plugin)->reset();
  }
  for (std::vector<std::shared_ptr<Layer>>::iterator filter = filters->begin();
    filter != filters->end(); ++filter)
  {
    (*filter)->reset();
  }
}

bool
Costmap2DROS::getRobotPose(geometry_msgs::msg::PoseStamped & global_pose)
{
  return nav2_util::getCurrentPose(
    global_pose, *tf_buffer_,
    global_frame_, robot_base_frame_, transform_tolerance_);
}

bool
Costmap2DROS::transformPoseToGlobalFrame(
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & transformed_pose)
{
  if (input_pose.header.frame_id == global_frame_) {
    transformed_pose = input_pose;
    return true;
  } else {
    return nav2_util::transformPoseInTargetFrame(
      input_pose, transformed_pose, *tf_buffer_,
      global_frame_, transform_tolerance_);
  }
}

rcl_interfaces::msg::SetParametersResult
Costmap2DROS::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  bool resize_map = false;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "robot_radius") {
        robot_radius_ = parameter.as_double();
        // Set the footprint
        if (use_radius_) {
          setRobotFootprint(makeFootprintFromRadius(robot_radius_));
        }
      } else if (name == "footprint_padding") {
        footprint_padding_ = parameter.as_double();
        padded_footprint_ = unpadded_footprint_;
        padFootprint(padded_footprint_, footprint_padding_);
        layered_costmap_->setFootprint(padded_footprint_);
      } else if (name == "transform_tolerance") {
        transform_tolerance_ = parameter.as_double();
      } else if (name == "publish_frequency") {
        map_publish_frequency_ = parameter.as_double();
        if (map_publish_frequency_ > 0) {
          publish_cycle_ = rclcpp::Duration::from_seconds(1 / map_publish_frequency_);
        } else {
          publish_cycle_ = rclcpp::Duration(-1s);
        }
      } else if (name == "resolution") {
        resize_map = true;
        resolution_ = parameter.as_double();
      } else if (name == "origin_x") {
        resize_map = true;
        origin_x_ = parameter.as_double();
      } else if (name == "origin_y") {
        resize_map = true;
        origin_y_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == "width") {
        if (parameter.as_int() > 0) {
          resize_map = true;
          map_width_meters_ = parameter.as_int();
        } else {
          RCLCPP_ERROR(
            get_logger(), "You try to set width of map to be negative or zero,"
            " this isn't allowed, please give a positive value.");
          result.successful = false;
          return result;
        }
      } else if (name == "height") {
        if (parameter.as_int() > 0) {
          resize_map = true;
          map_height_meters_ = parameter.as_int();
        } else {
          RCLCPP_ERROR(
            get_logger(), "You try to set height of map to be negative or zero,"
            " this isn't allowed, please give a positive value.");
          result.successful = false;
          return result;
        }
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "footprint") {
        footprint_ = parameter.as_string();
        std::vector<geometry_msgs::msg::Point> new_footprint;
        if (makeFootprintFromString(footprint_, new_footprint)) {
          setRobotFootprint(new_footprint);
        }
      } else if (name == "robot_base_frame") {
        // First, make sure that the transform between the robot base frame
        // and the global frame is available
        std::string tf_error;
        RCLCPP_INFO(get_logger(), "Checking transform");
        if (!tf_buffer_->canTransform(
            global_frame_, parameter.as_string(), tf2::TimePointZero,
            tf2::durationFromSec(1.0), &tf_error))
        {
          RCLCPP_WARN(
            get_logger(), "Timed out waiting for transform from %s to %s"
            " to become available, tf error: %s",
            parameter.as_string().c_str(), global_frame_.c_str(), tf_error.c_str());
          RCLCPP_WARN(
            get_logger(), "Rejecting robot_base_frame change to %s , leaving it to its original"
            " value of %s", parameter.as_string().c_str(), robot_base_frame_.c_str());
          result.successful = false;
          return result;
        }
        robot_base_frame_ = parameter.as_string();
      }
    }
  }

  if (resize_map && !layered_costmap_->isSizeLocked()) {
    layered_costmap_->resizeMap(
      (unsigned int)(map_width_meters_ / resolution_),
      (unsigned int)(map_height_meters_ / resolution_), resolution_, origin_x_, origin_y_);
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_costmap_2d
