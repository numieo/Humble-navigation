/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey */

#include "nav2_amcl/amcl_node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "message_filters/subscriber.h"
#include "nav2_amcl/angleutils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_util/string_utils.hpp"
#include "nav2_amcl/sensors/laser/laser.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

#include "nav2_amcl/portable_utils.hpp"

using namespace std::placeholders;
using rcl_interfaces::msg::ParameterType;
using namespace std::chrono_literals;

namespace nav2_amcl
{
using nav2_util::geometry_utils::orientationAroundZAxis;

AmclNode::AmclNode(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("amcl", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating");

  add_parameter(
    "alpha1", rclcpp::ParameterValue(0.2),
    "This is the alpha1 parameter", "These are additional constraints for alpha1");

  add_parameter(
    "alpha2", rclcpp::ParameterValue(0.2),
    "This is the alpha2 parameter", "These are additional constraints for alpha2");

  add_parameter(
    "alpha3", rclcpp::ParameterValue(0.2),
    "This is the alpha3 parameter", "These are additional constraints for alpha3");

  add_parameter(
    "alpha4", rclcpp::ParameterValue(0.2),
    "This is the alpha4 parameter", "These are additional constraints for alpha4");

  add_parameter(
    "alpha5", rclcpp::ParameterValue(0.2),
    "This is the alpha5 parameter", "These are additional constraints for alpha5");

  add_parameter(
    "base_frame_id", rclcpp::ParameterValue(std::string("base_footprint")),
    "Which frame to use for the robot base");

  add_parameter("beam_skip_distance", rclcpp::ParameterValue(0.5));
  add_parameter("beam_skip_error_threshold", rclcpp::ParameterValue(0.9));
  add_parameter("beam_skip_threshold", rclcpp::ParameterValue(0.3));
  add_parameter("do_beamskip", rclcpp::ParameterValue(false));

  add_parameter(
    "global_frame_id", rclcpp::ParameterValue(std::string("map")),
    "The name of the coordinate frame published by the localization system");

  add_parameter(
    "lambda_short", rclcpp::ParameterValue(0.1),
    "Exponential decay parameter for z_short part of model");

  add_parameter(
    "laser_likelihood_max_dist", rclcpp::ParameterValue(2.0),
    "Maximum distance to do obstacle inflation on map, for use in likelihood_field model");

  add_parameter(
    "laser_max_range", rclcpp::ParameterValue(100.0),
    "Maximum scan range to be considered",
    "-1.0 will cause the laser's reported maximum range to be used");

  add_parameter(
    "laser_min_range", rclcpp::ParameterValue(-1.0),
    "Minimum scan range to be considered",
    "-1.0 will cause the laser's reported minimum range to be used");

  add_parameter(
    "laser_model_type", rclcpp::ParameterValue(std::string("likelihood_field")),
    "Which model to use, either beam, likelihood_field, or likelihood_field_prob",
    "Same as likelihood_field but incorporates the beamskip feature, if enabled");

  add_parameter(
    "set_initial_pose", rclcpp::ParameterValue(false),
    "Causes AMCL to set initial pose from the initial_pose* parameters instead of "
    "waiting for the initial_pose message");

  add_parameter(
    "initial_pose.x", rclcpp::ParameterValue(0.0),
    "X coordinate of the initial robot pose in the map frame");

  add_parameter(
    "initial_pose.y", rclcpp::ParameterValue(0.0),
    "Y coordinate of the initial robot pose in the map frame");

  add_parameter(
    "initial_pose.z", rclcpp::ParameterValue(0.0),
    "Z coordinate of the initial robot pose in the map frame");

  add_parameter(
    "initial_pose.yaw", rclcpp::ParameterValue(0.0),
    "Yaw of the initial robot pose in the map frame");

  add_parameter(
    "max_beams", rclcpp::ParameterValue(60),
    "How many evenly-spaced beams in each scan to be used when updating the filter");

  add_parameter(
    "max_particles", rclcpp::ParameterValue(2000),
    "Maximum allowed number of particles");

  add_parameter(
    "min_particles", rclcpp::ParameterValue(500),
    "Minimum allowed number of particles");

  add_parameter(
    "odom_frame_id", rclcpp::ParameterValue(std::string("odom")),
    "Which frame to use for odometry");

  add_parameter("pf_err", rclcpp::ParameterValue(0.05));
  add_parameter("pf_z", rclcpp::ParameterValue(0.99));

  add_parameter(
    "recovery_alpha_fast", rclcpp::ParameterValue(0.0),
    "Exponential decay rate for the fast average weight filter, used in deciding when to recover "
    "by adding random poses",
    "A good value might be 0.1");

  add_parameter(
    "recovery_alpha_slow", rclcpp::ParameterValue(0.0),
    "Exponential decay rate for the slow average weight filter, used in deciding when to recover "
    "by adding random poses",
    "A good value might be 0.001");

  add_parameter(
    "resample_interval", rclcpp::ParameterValue(1),
    "Number of filter updates required before resampling");

  add_parameter("robot_model_type", rclcpp::ParameterValue("nav2_amcl::DifferentialMotionModel"));

  add_parameter(
    "save_pose_rate", rclcpp::ParameterValue(0.5),
    "Maximum rate (Hz) at which to store the last estimated pose and covariance to the parameter "
    "server, in the variables ~initial_pose_* and ~initial_cov_*. This saved pose will be used "
    "on subsequent runs to initialize the filter",
    "-1.0 to disable");

  add_parameter("sigma_hit", rclcpp::ParameterValue(0.2));

  add_parameter(
    "tf_broadcast", rclcpp::ParameterValue(true),
    "Set this to false to prevent amcl from publishing the transform between the global frame and "
    "the odometry frame");

  add_parameter(
    "transform_tolerance", rclcpp::ParameterValue(1.0),
    "Time with which to post-date the transform that is published, to indicate that this transform "
    "is valid into the future");

  add_parameter(
    "update_min_a", rclcpp::ParameterValue(0.2),
    "Rotational movement required before performing a filter update");

  add_parameter(
    "update_min_d", rclcpp::ParameterValue(0.25),
    "Translational movement required before performing a filter update");

  add_parameter("z_hit", rclcpp::ParameterValue(0.5));
  add_parameter("z_max", rclcpp::ParameterValue(0.05));
  add_parameter("z_rand", rclcpp::ParameterValue(0.5));
  add_parameter("z_short", rclcpp::ParameterValue(0.05));

  add_parameter(
    "always_reset_initial_pose", rclcpp::ParameterValue(false),
    "Requires that AMCL is provided an initial pose either via topic or initial_pose* parameter "
    "(with parameter set_initial_pose: true) when reset. Otherwise, by default AMCL will use the"
    "last known pose to initialize");

  add_parameter(
    "scan_topic", rclcpp::ParameterValue("scan"),
    "Topic to subscribe to in order to receive the laser scan for localization");

  add_parameter(
    "map_topic", rclcpp::ParameterValue("map"),
    "Topic to subscribe to in order to receive the map to localize on");

  add_parameter(
    "first_map_only", rclcpp::ParameterValue(false),
    "Set this to true, when you want to load a new map published from the map_server");
}

AmclNode::~AmclNode()
{
}

nav2_util::CallbackReturn
AmclNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");  // 表示AMCL正在进行配置
  callback_group_ = create_callback_group(   // 创建一个回调组（Callback Group），用于管理和控制回调函数的执行。这里使用MutuallyExclusive类型的回调组，表示回调函数将在互斥的上下文中执行，即同一时刻只会有一个回调函数执行
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  initParameters();          // 初始化AMCL的参数
  initTransforms();          // 初始化坐标变换（Transform）的配置，用于将不同传感器的数据转换到统一的坐标系
  initParticleFilter();      // 初始化粒子滤波器，AMCL使用粒子滤波器来估计机器人的位姿
  initLaserScan();           // 初始化激光扫描数据的配置，通常激光扫描用于进行环境感知
  initMessageFilters();      // 初始化用于过滤和同步消息的配置，可能涉及多个传感器数据的整合
  initPubSub();              // 初始化发布和订阅的配置，用于与其他ROS节点进行通信
  initServices();            // 初始化提供的服务的配置，用于接受其他节点的请求
  initOdometry();            // 初始化里程计（Odometry）数据的配置，里程计通常用于估计机器人的运动
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();  // 创建一个单线程执行器，用于按顺序执行回调函数
  executor_->add_callback_group(callback_group_, get_node_base_interface());  // 将之前创建的回调组添加到执行器中，以便在执行回调函数时使用
  executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);      // 创建一个线程，用于在单独的线程中运行执行器，从而避免阻塞主线程
  return nav2_util::CallbackReturn::SUCCESS;                                  // 返回一个成功状态，表示初始化和配置完成
}

nav2_util::CallbackReturn
AmclNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  // 表示AMCL节点正在激活
  RCLCPP_INFO(get_logger(), "Activating");

  // Lifecycle publishers must be explicitly activated
  // 激活节点的发布者，在ROS 2生命周期中，需要显式激活发布者以开始发布消息
  pose_pub_->on_activate();
  particle_cloud_pub_->on_activate();

  first_pose_sent_ = false; // 将一个变量 first_pose_sent_ 设置为 false，用于跟踪是否已经发送了第一个位姿
  // Keep track of whether we're in the active state. We won't
  // process incoming callbacks until we are
  active_ = true;

  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    // ... 创建并初始化一个位姿消息 ...
    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation = orientationAroundZAxis(initial_pose_yaw_);
    // 调用 initialPoseReceived 函数进行处理
    initialPoseReceived(msg);
  } else if (init_pose_received_on_inactive) {
    // 在非激活状态下收到了初始位姿信息（init_pose_received_on_inactive 为 true），则调用 handleInitialPose 函数处理该初始位姿。
    handleInitialPose(last_published_pose_);
  }

  // 创建一个名为 node 的智能指针，指向当前的 AMCL 节点，这是为了在后续操作中能够使用智能指针来管理节点的生命周期
  auto node = shared_from_this();
  // Add callback for dynamic parameters
  // 添加了一个回调函数，用于在节点的参数动态更新时触发。add_on_set_parameters_callback 函数
  // 允许你指定一个回调函数，该函数将在节点的参数被设置或更新时被调用。在这里，回调函数被绑定到了 
  // dynamicParametersCallback 函数，并且使用 std::placeholders::_1 来表示参数的占位符
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &AmclNode::dynamicParametersCallback,
      this, std::placeholders::_1));

  // create bond connection
  // 调用了一个函数 createBond()，用于创建与其他节点之间的绑定（bonding）连接。
  // 绑定是一种机制，用于确保节点之间的通信和协作，以及在节点失效时能够进行适当的处理
  createBond();
  // 返回一个成功状态，表示这个生命周期回调函数已经执行完毕
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AmclNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  // 表示AMCL节点正在停用
  RCLCPP_INFO(get_logger(), "Deactivating");
  // 表示节点已经不再处于激活状态
  active_ = false;

  // Lifecycle publishers must be explicitly deactivated
  // 显式停用发布者以停止发布消息
  pose_pub_->on_deactivate();
  particle_cloud_pub_->on_deactivate();

  // reset dynamic parameter handler
  // 重置动态参数处理器，可能是清除参数的设置或更新回调函数，以确保在节点停用时不再处理参数更新
  dyn_params_handler_.reset();

  // destroy bond connection
  // 销毁与其他节点之间的绑定（bonding）连接
  destroyBond();
  // 返回一个成功状态，表示这个生命周期回调函数已经执行完毕
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AmclNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  // 表示AMCL节点正在进行清理操作
  RCLCPP_INFO(get_logger(), "Cleaning up");
  // 重置执行器线程，这可能是用来终止执行器线程的运行，以便在清理过程中不再执行任何回调函数
  executor_thread_.reset();

  // Get rid of the inputs first (services and message filter input), so we
  // don't continue to process incoming messages
  // 重置和释放
  global_loc_srv_.reset();              // 全局定位服务
  nomotion_update_srv_.reset();         // 无运动更新服务
  initial_pose_sub_.reset();            // 初始位姿订阅
  laser_scan_connection_.disconnect();  // 激光扫描连接
  laser_scan_filter_.reset();           // 激光扫描过滤器
  laser_scan_sub_.reset();              // 激光扫描订阅

  // Map
  // 如果存在地图资源，释放地图资源，然后将指向地图的指针设置为 nullptr
  if (map_ != NULL) {
    map_free(map_);
    map_ = nullptr;
  }
  // 将标志变量 first_map_received_ 设置为 false，表示没有收到地图。
  // 同时，将一个记录自由空间索引的向量的大小设置为 0，以释放相关资源。
  first_map_received_ = false;
  free_space_indices.resize(0);

  // Transforms
  // 释放与坐标转换相关的资源，包括发布者、监听器和缓冲区，通过 reset() 函数将智能指针置为空
  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  // PubSub
  // 释放与发布订阅相关的资源，包括位姿发布者和粒子云发布者，通过 reset() 函数将智能指针置为空
  pose_pub_.reset();
  particle_cloud_pub_.reset();

  // Odometry
  // 释放与里程计模型相关的资源，通过 reset() 函数将智能指针置为空
  motion_model_.reset();

  // Particle Filter
  // 释放粒子滤波器相关的资源，通过调用 pf_free 函数释放滤波器，然后将指向滤波器的指针设置为 nullptr
  pf_free(pf_);
  pf_ = nullptr;

  // Laser Scan
  // 清理与激光扫描相关的资源和状态，包括激光传感器列表、激光更新列表、
  // 坐标系到激光传感器的映射等，然后将强制更新标志 force_update_ 设置为 true
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();
  force_update_ = true;

  // 用于在节点配置或激活阶段，如果需要设置初始位姿，则将初始位姿的参数设置到节点的参数服务器中
  if (set_initial_pose_) {
    set_parameter(
      rclcpp::Parameter(  // rclcpp::Parameter 用于创建一个参数对象，包括参数名和参数值
        "initial_pose.x",           // rclcpp::ParameterValue 用于将实际值转换为参数值
        rclcpp::ParameterValue(last_published_pose_.pose.pose.position.x)));
    set_parameter(
      rclcpp::Parameter(
        "initial_pose.y",
        rclcpp::ParameterValue(last_published_pose_.pose.pose.position.y)));
    set_parameter(
      rclcpp::Parameter(
        "initial_pose.z",
        rclcpp::ParameterValue(last_published_pose_.pose.pose.position.z)));
    set_parameter(
      rclcpp::Parameter(
        "initial_pose.yaw",
        rclcpp::ParameterValue(tf2::getYaw(last_published_pose_.pose.pose.orientation))));
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AmclNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool   // 用于检查给定的时间间隔内是否已经过了足够的时间
AmclNode::checkElapsedTime(std::chrono::seconds check_interval, rclcpp::Time last_time)
{   
  // check_interval 是一个表示检查的时间间隔的 std::chrono::seconds 类型，
  // last_time 是一个 rclcpp::Time 类型的时间点，表示上一次的时间

  // 计算当前时间与上一次时间 last_time 之间的时间差，得到一个 rclcpp::Duration 对象 elapsed_time
  rclcpp::Duration elapsed_time = now() - last_time;

  // 将 elapsed_time 转换为秒（通过将纳秒除以 1e9），然后与 check_interval 进行比较。
  // 如果已经过了足够的时间，即当前时间与上一次时间差大于指定的时间间隔，那么函数返回 true，否则返回 false
  if (elapsed_time.nanoseconds() * 1e-9 > check_interval.count()) {
    return true;
  }
  return false;
}

#if NEW_UNIFORM_SAMPLING
std::vector<std::pair<int, int>> AmclNode::free_space_indices;
#endif

bool
AmclNode::getOdomPose(
  // odom_pose 是一个用于接收机器人位姿信息的 geometry_msgs::msg::PoseStamped 类型的消息
  geometry_msgs::msg::PoseStamped & odom_pose,
  double & x, double & y, double & yaw,
  // sensor_timestamp 是用于时间戳的 rclcpp::Time 类型, frame_id 是指定帧的标识符
  const rclcpp::Time & sensor_timestamp, const std::string & frame_id)
{
  // Get the robot's pose
  // 存储机器人的位姿信息
  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = nav2_util::strip_leading_slash(frame_id);
  ident.header.stamp = sensor_timestamp;
  // 创建一个单位变换，将其转换为 ident.pose，表示机器人的位姿。这里的 ident.pose 将包含位置和方向
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

  // try 块尝试将 ident 中的位姿信息从一个帧变换到另一个帧（从 frame_id 到 odom_frame_id_）
  try {
    tf_buffer_->transform(ident, odom_pose, odom_frame_id_);
  } catch (tf2::TransformException & e) {
    // 如果变换过程中出现异常（tf2::TransformException），表示变换失败。
    // 在这种情况下，scan_error_count_ 会增加，用于计算连续失败的变换次数
    ++scan_error_count_;
    if (scan_error_count_ % 20 == 0) {
      // 如果连续失败次数达到了一定阈值（每 20 次失败），则会输出一条错误日志，显示连续变换失败的次数和具体错误信息
      RCLCPP_ERROR(
        get_logger(), "(%d) consecutive laser scan transforms failed: (%s)", scan_error_count_,
        e.what());
    }
    // 如果变换失败，函数会返回 false，表示获取里程计位姿信息失败
    return false;
  }
  // 如果变换成功，将连续变换失败次数重置为 0，因为已经成功获取了一个有效的变换
  scan_error_count_ = 0;  // reset since we got a good transform
  // 将获取到的里程计位姿信息填充到传递的参数 x、y 和 yaw 中
  x = odom_pose.pose.position.x;
  y = odom_pose.pose.position.y;
  yaw = tf2::getYaw(odom_pose.pose.orientation);
  // 函数返回 true，表示成功获取了里程计位姿信息
  return true;
}

pf_vector_t  // 在粒子滤波器中生成均匀分布的粒子位姿, 用于采样机器人的初始粒子位姿
AmclNode::uniformPoseGenerator(void * arg)
{
  // 这个函数接受一个 void* 类型的指针参数 arg，然后将其转换为 map_t* 类型，这个指针指向一个地图对象
  map_t * map = reinterpret_cast<map_t *>(arg);

#if NEW_UNIFORM_SAMPLING
  // 随机选择一个索引 rand_index，范围在 0 到 free_space_indices 数组大小之间
  unsigned int rand_index = drand48() * free_space_indices.size();
  // 从 free_space_indices 数组中获取随机索引处的自由空间坐标 free_point，
  // 在自由空间中随机选取一个点作为粒子的初始位置
  std::pair<int, int> free_point = free_space_indices[rand_index];
  pf_vector_t p;  // 粒子位姿
  // 将 free_point 中的坐标转换为地图坐标系中的位置，
  // 并将结果分别赋值给 p.v[0] 和 p.v[1]，这表示粒子的 x 和 y 坐标
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  // 生成一个随机的角度，将其限制在 -π 到 π 之间，并将结果赋值给 p.v[2]，这表示粒子的角度
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;
  // 计算地图的 x、y 范围，这些范围将用于限制生成的粒子的 x 和 y 坐标
  min_x = (map->size_x * map->scale) / 2.0 - map->origin_x;
  max_x = (map->size_x * map->scale) / 2.0 + map->origin_x;
  min_y = (map->size_y * map->scale) / 2.0 - map->origin_y;
  max_y = (map->size_y * map->scale) / 2.0 + map->origin_y;
  // 表示生成的粒子位姿
  pf_vector_t p;
  // 输出一条调试日志，表示正在生成新的均匀样本
  RCLCPP_DEBUG(get_logger(), "Generating new uniform sample");
  // 使用循环不断生成粒子位姿，直到找到一个符合条件的位姿
  for (;; ) {
    // 随机生成一个角度，将其限制在 -π 到 π 之间
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    // 检查生成的粒子位姿对应的地图单元是否是自由（没有障碍物）的。如果是，循环结束
    int i, j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if (MAP_VALID(map, i, j) && (map->cells[MAP_INDEX(map, i, j)].occ_state == -1)) {
      break;
    }
  }
#endif
  // 生成的粒子位姿 p 将被返回
  return p;
}

void        // 处理全局定位请求的回调函数
AmclNode::globalLocalizationCallback(
  // 回调函数的参数包括请求的请求头、请求对象和响应对象。
  // 在这里的代码中，这些参数都没有被使用，因此在参数名后面用注释标明
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  // 使用 std::lock_guard 创建一个互斥锁，以确保在回调函数执行期间不会发生竞态条件
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  // 表示正在使用均匀分布初始化
  RCLCPP_INFO(get_logger(), "Initializing with uniform distribution");
  // 使用均匀分布的粒子位姿初始化粒子滤波器
  // pf_ 是粒子滤波器对象, 第二个参数是一个函数指针，指向 uniformPoseGenerator 函数，
  // 用于在粒子滤波器中生成均匀分布的粒子位姿
  pf_init_model(
    pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
    // void* 指针，指向地图对象
    reinterpret_cast<void *>(map_));
  // 表示全局初始化已完成
  RCLCPP_INFO(get_logger(), "Global initialisation done!");
  // 将 initial_pose_is_known_ 标志设置为 true，表示初始化的初始位姿已知
  initial_pose_is_known_ = true;
  // 将 pf_init_ 标志设置为 false，表示粒子滤波器的初始化已完成
  pf_init_ = false;
}

// force nomotion updates (amcl updating without requiring motion)
void    // 用于处理无运动更新请求的回调函数
AmclNode::nomotionUpdateCallback(
  // 回调函数的参数包括请求的请求头、请求对象和响应对象。这些参数都没有被使用，因此在参数名后面用注释标明
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{ // 表示正在请求无运动更新
  RCLCPP_INFO(get_logger(), "Requesting no-motion update");
  // 将 force_update_ 标志设置为 true，表示请求进行无运动更新。这可能会触发粒子滤波器进行更新操作，即使机器人没有实际运动
  force_update_ = true;
}

void    // 用于处理接收到的初始位姿信息的函数
AmclNode::initialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // 使用 std::lock_guard 创建一个互斥锁，以确保在处理初始位姿信息时不会发生竞态条件
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  // 表示接收到了初始位姿信息
  RCLCPP_INFO(get_logger(), "initialPoseReceived");

  if (msg->header.frame_id == "") {
    // This should be removed at some point
    // 检查接收到的初始位姿消息的帧标识符是否为空
    RCLCPP_WARN(
      get_logger(),
      "Received initial pose with empty frame_id. You should always supply a frame_id.");
    return;
  }
  if (nav2_util::strip_leading_slash(msg->header.frame_id) != global_frame_id_) {
    // 检查接收到的初始位姿消息的帧标识符是否与全局帧标识符一致
    // 如果不一致，输出一个警告日志，表示忽略在非全局帧中的初始位姿，因为初始位姿必须位于全局帧中
    RCLCPP_WARN(
      get_logger(),
      "Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
      nav2_util::strip_leading_slash(msg->header.frame_id).c_str(),
      global_frame_id_.c_str());
    return;
  }
  // Overriding last published pose to initial pose
  // 将接收到的初始位姿消息 msg 赋值给变量 last_published_pose_，用于记录最后发布的位姿
  last_published_pose_ = *msg;
  // 检查 AMCL 是否处于活动状态（active_ 标志）
  if (!active_) {
    // 如果不是活动状态，将 init_pose_received_on_inactive 标志设置为 true，表示在非活动状态下接收到初始位姿请求
    init_pose_received_on_inactive = true;
    // 输出一个警告日志，表示接收到初始位姿请求，但是 AMCL 还未处于活动状态
    RCLCPP_WARN(
      get_logger(), "Received initial pose request, "
      "but AMCL is not yet in the active state");
    // 在这种情况下，函数返回，不继续处理初始位姿
    return;
  }
  // 如果 AMCL 处于活动状态，调用 handleInitialPose 函数来处理接收到的初始位姿信息。这个函数将使用初始位姿信息来设置粒子滤波器的状态
  handleInitialPose(*msg);
}

void    // 处理接收到的初始位姿信息并更新粒子滤波器的状态
AmclNode::handleInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{ // 首先尝试获取当前机器人的初始位姿，然后使用这个位姿来更新粒子滤波器的状态
  // 使用 std::lock_guard 创建一个互斥锁，以确保在处理初始位姿信息时不会发生竞态条件
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  geometry_msgs::msg::TransformStamped tx_odom;
  try {
    rclcpp::Time rclcpp_time = now();
    tf2::TimePoint tf2_time(std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

    // Check if the transform is available
    // 通过 tf_buffer_ 对象查找一个变换，将时间戳转换为适合的时间
    // tx_odom 将存储从初始位姿帧到里程计帧的变换
    tx_odom = tf_buffer_->lookupTransform(
      base_frame_id_, tf2_ros::fromMsg(msg.header.stamp),
      base_frame_id_, tf2_time, odom_frame_id_);
  } catch (tf2::TransformException & e) { // 如果获取变换时出现异常（tf2::TransformException），表示变换失败
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    // 如果 sent_first_transform_ 为真，输出一个警告日志，表示初始位姿的变换失败。
    // 这可能会在启动时发生，因为全局帧可能尚不存在。对于在运动中设置位姿时的实时变换，
    // 我们不会特别关心这个启动条件，因此忽略它不会对我们产生太多影响
    if (sent_first_transform_) {
      RCLCPP_WARN(get_logger(), "Failed to transform initial pose in time (%s)", e.what());
    }
    // 如果变换失败，将一个单位变换矩阵赋值给 tx_odom.transform，以确保我们有一个初始的变换
    tf2::impl::Converter<false, true>::convert(tf2::Transform::getIdentity(), tx_odom.transform);
  }
  // 将初始位姿信息转换并更新粒子滤波器的状态
  // 将 tx_odom.transform 转换为 tf2::Transform 类型的 tx_odom_tf2 变量
  tf2::Transform tx_odom_tf2;
  tf2::impl::Converter<true, false>::convert(tx_odom.transform, tx_odom_tf2);
  // 将接收到的初始位姿消息中的位姿部分 msg.pose.pose 转换为 tf2::Transform 类型的 pose_old 变量
  tf2::Transform pose_old;
  tf2::impl::Converter<true, false>::convert(msg.pose.pose, pose_old);
  // 计算新的位姿 pose_new，它是将 pose_old 与 tx_odom_tf2 组合后的结果
  tf2::Transform pose_new = pose_old * tx_odom_tf2;

  // Transform into the global frame
  // 表示正在设置新的位姿, 打印当前时间戳以及位姿的 x、y 坐标和方向（角度）
  RCLCPP_INFO(
    get_logger(), "Setting pose (%.6f): %.3f %.3f %.3f",
    now().nanoseconds() * 1e-9,
    pose_new.getOrigin().x(),
    pose_new.getOrigin().y(),
    tf2::getYaw(pose_new.getRotation()));

  // Re-initialize the filter
  // 重新初始化粒子滤波器
  // 创建一个 pf_vector_t 类型的变量 pf_init_pose_mean，将位姿信息的 x、y 坐标和方向设置为初始均值
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = tf2::getYaw(pose_new.getRotation());
  // 创建一个 pf_matrix_t 类型的变量 pf_init_pose_cov，将位姿信息的协方差矩阵的前两行前两列设置为初始协方差
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  // 通过迭代复制位姿信息的协方差矩阵中的值，从 6 维转换为 3 维
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      pf_init_pose_cov.m[i][j] = msg.pose.covariance[6 * i + j];
    }
  }
  // 将位姿信息中的协方差矩阵的最后一个元素（角度方向的方差）设置为粒子滤波器初始化位姿的协方差
  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6 * 5 + 5];
  // 将重新初始化的初始位姿均值和协方差矩阵传递给粒子滤波器，以重新初始化粒子滤波器的状态
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  // 表示粒子滤波器已经完成了初始化
  pf_init_ = false;
  // 表示在非活动状态下接收到初始位姿的请求已经被处理
  init_pose_received_on_inactive = false;
  // 表示初始位姿已经被知晓
  initial_pose_is_known_ = true;
}

void    // 处理接收到的激光扫描数据
AmclNode::laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan)
{
  //  创建一个互斥锁，以确保在处理激光扫描数据时不会发生竞态条件
  std::lock_guard<std::recursive_mutex> cfl(mutex_);

  // Since the sensor data is continually being published by the simulator or robot,
  // we don't want our callbacks to fire until we're in the active state
  if (!active_) {return;}   // 在机器人还没有进入活动状态时，直接返回，不进行激光扫描数据的处理
  // 如果尚未收到地图数据 (first_map_received_ 为 false)，则输出一条警告信息表示正在等待地图
  if (!first_map_received_) {
    if (checkElapsedTime(2s, last_time_printed_msg_)) {
    //  如果距离上次输出消息的时间间隔大于 2 秒，会输出一条等待地图的警告消息
      RCLCPP_WARN(get_logger(), "Waiting for map....");
      last_time_printed_msg_ = now();
    }
    // 如果还没有收到地图数据，直接返回，不进行激光扫描数据的处理
    return;
  }
  // 获取激光扫描消息的帧标识符，存储在 laser_scan_frame_id 变量中
  std::string laser_scan_frame_id = nav2_util::strip_leading_slash(laser_scan->header.frame_id);
  last_laser_received_ts_ = now();  // 记录当前时间戳为最后接收到激光数据的时间戳
  // 创建一个整数变量 laser_index，并初始化为 -1，用于存储当前激光扫描数据在激光传感器数组中的索引
  int laser_index = -1;
  // 存储激光传感器的位姿信息
  geometry_msgs::msg::PoseStamped laser_pose;

  // 判断是否已经获取了从机器人底盘坐标系到激光传感器坐标系的变换
  // Do we have the base->base_laser Tx yet?
  // 使用 frame_to_laser_ 容器查找是否已经存在 laser_scan_frame_id 对应的变换信息
  if (frame_to_laser_.find(laser_scan_frame_id) == frame_to_laser_.end()) {
    // 如果不存在，表示还没有获取该变换信息，则调用 addNewScanner 函数尝试添加新的激光传感器，如果添加失败则直接返回
    if (!addNewScanner(laser_index, laser_scan, laser_scan_frame_id, laser_pose)) {
      return;  // could not find transform
    }
  } else {
    // we have the laser pose, retrieve laser index
    // 如果存在，表示已经有了机器人的激光传感器的位姿变换信息，则通过查找 frame_to_laser_ 容器获取 laser_scan_frame_id 对应的激光传感器索引，存储在 laser_index 变量中
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }

  // Where was the robot when this scan was taken?
  // 获取机器人在进行激光扫描时的位姿
  pf_vector_t pose;
  if (!getOdomPose(
      // 调用 getOdomPose 函数，将机器人的最新里程计位姿 latest_odom_pose_ 转换为激光数据时间戳对应的位姿信息，
      // 并将 x、y 坐标和方向分别存储在 pose.v[0]、pose.v[1] 和 pose.v[2] 中
      latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
      laser_scan->header.stamp, base_frame_id_))
  {
    // 如果无法获取位姿，输出一个错误日志，表示无法确定与激光扫描关联的机器人位姿，然后直接返回
    RCLCPP_ERROR(get_logger(), "Couldn't determine robot's pose associated with laser scan");
    return;
  }
  
  pf_vector_t delta = pf_vector_zero();   // 存储位姿的变化量
  bool force_publication = false;   // 标记是否需要强制发布粒子滤波器的结果
  if (!pf_init_) {                  // 如果粒子滤波器尚未初始化
    // Pose at last filter update
    pf_odom_pose_ = pose;           // 设置为当前位姿 pose
    pf_init_ = true;

    for (unsigned int i = 0; i < lasers_update_.size(); i++) {
      lasers_update_[i] = true;   // 将所有激光传感器的更新标志 lasers_update_ 设置为 true，表示所有激光传感器都需要进行更新
    }

    force_publication = true;     // 表示需要强制发布粒子滤波器的结果
    resample_count_ = 0;          // 将重采样计数器 resample_count_ 设置为 0
  } else {                        // 如果粒子滤波器已经初始化
    // Set the laser update flags
    if (shouldUpdateFilter(pose, delta)) {  // 判断是否需要更新粒子滤波器
      for (unsigned int i = 0; i < lasers_update_.size(); i++) {
        lasers_update_[i] = true;   // 如果需要更新，将所有激光传感器的更新标志 lasers_update_ 设置为 true
      }
    }
    if (lasers_update_[laser_index]) {  // 如果当前激光传感器需要更新
      motion_model_->odometryUpdate(pf_, pose, delta);  // 对粒子滤波器进行里程计更新
    }
    force_update_ = false;            // 不需要强制更新
  }

  bool resampled = false;         // 标记是否已经进行了粒子重采样

  // If the robot has moved, update the filter
  if (lasers_update_[laser_index]) {    // 如果当前激光传感器需要进行更新
    updateFilter(laser_index, laser_scan, pose);  // 根据激光数据更新粒子滤波器

    // Resample the particles
    // 如果满足重采样条件 (resample_count_ 除以 resample_interval_ 的余数为 0)，
    // 则调用 pf_update_resample 函数进行粒子重采样
    if (!(++resample_count_ % resample_interval_)) {
      pf_update_resample(pf_);
      resampled = true; // 表示已经进行了粒子重采样
    }
    // 获取当前粒子滤波器中的样本集合信息
    pf_sample_set_t * set = pf_->sets + pf_->current_set;
    // 输出当前样本集合中的样本数量
    RCLCPP_DEBUG(get_logger(), "Num samples: %d\n", set->sample_count);
    // 如果不需要强制更新 (force_update_ 为 false)，则发布粒子云信息
    if (!force_update_) {
      publishParticleCloud(set);
    }
  }
  // 如果已经进行了粒子重采样, 或者需要强制发布, 或者尚未发送过第一个位姿估计
  if (resampled || force_publication || !first_pose_sent_) {
    amcl_hyp_t max_weight_hyps;
    std::vector<amcl_hyp_t> hyps;   // 存储多个估计假设的向量
    int max_weight_hyp = -1;        // 存储具有最大权重的估计假设的索引 
    // 获取具有最大权重的估计假设，并将结果存储在 hyps 和 max_weight_hyps 中
    if (getMaxWeightHyp(hyps, max_weight_hyps, max_weight_hyp)) {
      // 发布机器人的估计位姿，该位姿基于具有最大权重的估计假设
      publishAmclPose(laser_scan, hyps, max_weight_hyp);
      // 计算从地图坐标系到里程计坐标系的变换，该变换基于具有最大权重的估计假设
      calculateMaptoOdomTransform(laser_scan, hyps, max_weight_hyp);
      // 如果需要广播变换
      if (tf_broadcast_ == true) {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        // 将激光扫描消息的时间戳转换为 tf2 时间
        auto stamp = tf2_ros::fromMsg(laser_scan->header.stamp);
        // 计算一个变换的过期时间点，用于在给定的容忍时间内使用变换
        tf2::TimePoint transform_expiration = stamp + transform_tolerance_;
        // 发送地图到里程计的变换，该变换在指定的过期时间点前有效
        sendMapToOdomTransform(transform_expiration);
        sent_first_transform_ = true;
      }
    } else {
      // 如果无法获取估计位姿的假设，则输出一个错误日志
      RCLCPP_ERROR(get_logger(), "No pose!");
    }
  } else if (latest_tf_valid_) {  // 如果之前的条件都没有满足，但是最新的变换信息仍然有效
    if (tf_broadcast_ == true) {  // 如果需要广播变换
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      // 将激光扫描消息的时间戳转换为 tf2 时间
      tf2::TimePoint transform_expiration = tf2_ros::fromMsg(laser_scan->header.stamp) +
        transform_tolerance_;
      // 发送地图到里程计的变换，该变换在指定的过期时间点前有效
      sendMapToOdomTransform(transform_expiration);
    }
  }
}
// 将新的激光扫描器添加到系统中。它会创建新的激光对象，更新相关的数据结构，
// 执行激光位姿的坐标变换，将激光位姿信息设置给对应的激光对象，并将激光帧 ID 与
// 激光索引进行映射。如果在坐标变换过程中出现错误，函数将返回 false，否则返回 true 
// 表示成功添加了新的激光扫描器
bool AmclNode::addNewScanner(
  int & laser_index,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::string & laser_scan_frame_id,
  geometry_msgs::msg::PoseStamped & laser_pose)
{
  // 创建一个新的激光对象并更新相关数据结构
  lasers_.push_back(createLaserObject()); // 创建新的激光对象
  lasers_update_.push_back(true);         // 初始化激光更新标志为 true
  laser_index = frame_to_laser_.size();   // 记录激光索引

  // 获取激光在其自身坐标系中的位姿
  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = laser_scan_frame_id;
  ident.header.stamp = rclcpp::Time();
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
  // 将激光位姿变换到基础框架
  try {
    tf_buffer_->transform(ident, laser_pose, base_frame_id_, transform_tolerance_);
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(
      get_logger(), "Couldn't transform from %s to %s, "
      "even though the message notifier is in use: (%s)",
      laser_scan->header.frame_id.c_str(),
      base_frame_id_.c_str(), e.what());
    return false;
  }
  // 将激光位姿转换为pf_vector_t类型
  pf_vector_t laser_pose_v;
  laser_pose_v.v[0] = laser_pose.pose.position.x;
  laser_pose_v.v[1] = laser_pose.pose.position.y;
  // laser mounting angle gets computed later -> set to 0 here!
  // 激光安装角度稍后计算，所以这里设置为0
  laser_pose_v.v[2] = 0;
  lasers_[laser_index]->SetLaserPose(laser_pose_v);
  // 存储帧id和激光索引之间的映射
  frame_to_laser_[laser_scan->header.frame_id] = laser_index;
  return true; // 成功添加激光扫描器时返回 true
}

bool AmclNode::shouldUpdateFilter(const pf_vector_t pose, pf_vector_t & delta)
{
  // 计算自上次过滤器更新以来姿态（位置和方向）的变化
  delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];  // x 位移变化
  delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];  // y 位移变化
  delta.v[2] = angleutils::angle_diff(pose.v[2], pf_odom_pose_.v[2]); // 角度变化，考虑周期性

  // See if we should update the filter
  // 根据计算出的变化确定过滤器是否应该更新
  bool update = fabs(delta.v[0]) > d_thresh_ || // x 位移变化是否大于阈值
    fabs(delta.v[1]) > d_thresh_ ||   // y 位移变化是否大于阈值
    fabs(delta.v[2]) > a_thresh_;     // 角度变化是否大于阈值
  update = update || force_update_;   // 强制更新标志是否为 true
  return update;                      // 返回是否应该更新滤波器状态的布尔值
}

bool AmclNode::updateFilter(
  const int & laser_index,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const pf_vector_t & pose)
{
  // 创建一个 LaserData 结构体来存储激光数据
  nav2_amcl::LaserData ldata;
  ldata.laser = lasers_[laser_index]; // 设置激光对象
  ldata.range_count = laser_scan->ranges.size();   // 设置激光扫描数据的点数
  // To account for lasers that are mounted upside-down, we determine the
  // min, max, and increment angles of the laser in the base frame.
  //
  // Construct min and max angles of laser, in the base_link frame.
  // Here we set the roll pich yaw of the lasers.  We assume roll and pich are zero.

  // 为了考虑激光器可能是倒置安装的情况，我们在基底坐标系中确定激光器的最小、最大和增量角度。
  // 构建激光器在基底坐标系中的最小和最大角度。
  // 在这里我们设置激光器的滚动、俯仰角为零。
  geometry_msgs::msg::QuaternionStamped min_q, inc_q;
  min_q.header.stamp = laser_scan->header.stamp;
  min_q.header.frame_id = nav2_util::strip_leading_slash(laser_scan->header.frame_id);
  min_q.quaternion = orientationAroundZAxis(laser_scan->angle_min); // 设置激光器的最小角度

  inc_q.header = min_q.header;
  // 设置激光器的增量角度
  inc_q.quaternion = orientationAroundZAxis(laser_scan->angle_min + laser_scan->angle_increment);
  try {
    // 将最小和增量角度从激光扫描帧转换到基底坐标系
    tf_buffer_->transform(min_q, min_q, base_frame_id_);
    tf_buffer_->transform(inc_q, inc_q, base_frame_id_);
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(
      get_logger(), "Unable to transform min/max laser angles into base frame: %s",
      e.what());
    return false; // 如果发生变换错误，返回 false
  }
  // 获取变换后的最小角度和增量角度
  double angle_min = tf2::getYaw(min_q.quaternion);
  double angle_increment = tf2::getYaw(inc_q.quaternion) - angle_min;

  // wrapping angle to [-pi .. pi]
  // 角度范围调整到 [-pi .. pi] 之间
  angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;

  RCLCPP_DEBUG(
    get_logger(), "Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min,
    angle_increment);

  // Apply range min/max thresholds, if the user supplied them
  // 如果用户提供了激光最大测量范围阈值
  if (laser_max_range_ > 0.0) {
    // 将激光扫描数据的最大测量范围与用户提供的阈值进行比较，取较小值作为实际范围
    ldata.range_max = std::min(laser_scan->range_max, static_cast<float>(laser_max_range_));
  } else {
    ldata.range_max = laser_scan->range_max;  // 否则使用激光扫描数据中的最大测量范围
  }
  double range_min;
  // 如果用户提供了激光最小测量范围阈值
  if (laser_min_range_ > 0.0) {
    // 将激光扫描数据的最小测量范围与用户提供的阈值进行比较，取较大值作为实际范围
    range_min = std::max(laser_scan->range_min, static_cast<float>(laser_min_range_));
  } else {
    range_min = laser_scan->range_min;  // 否则使用激光扫描数据中的最小测量范围
  }

  // The LaserData destructor will free this memory
  // 为激光测量数据分配内存
  ldata.ranges = new double[ldata.range_count][2];
  // 对于小于等于最小测量范围的测量值，映射为最大测量范围
  for (int i = 0; i < ldata.range_count; i++) {
    // amcl doesn't (yet) have a concept of min range.  So we'll map short
    // readings to max range.
    if (laser_scan->ranges[i] <= range_min) {
      ldata.ranges[i][0] = ldata.range_max;
    } else {
      ldata.ranges[i][0] = laser_scan->ranges[i];
    }
    // Compute bearing
    // 计算角度（bearing）
    ldata.ranges[i][1] = angle_min +
      (i * angle_increment);
  }
  // 进行激光传感器更新
  lasers_[laser_index]->sensorUpdate(pf_, reinterpret_cast<nav2_amcl::LaserData *>(&ldata));
  lasers_update_[laser_index] = false;  // 将激光传感器更新标志设置为 false
  pf_odom_pose_ = pose;   // 更新上一次的里程计姿态
  return true;
}

void
AmclNode::publishParticleCloud(const pf_sample_set_t * set)
{
  // If initial pose is not known, AMCL does not know the current pose
  // 如果初始姿态未知，AMCL 将不知道当前姿态，所以不进行发布
  if (!initial_pose_is_known_) {return;}
  // 创建一个消息来发布带权重的粒子云
  auto cloud_with_weights_msg = std::make_unique<nav2_msgs::msg::ParticleCloud>();
  // 设置消息的时间戳为当前时间
  cloud_with_weights_msg->header.stamp = this->now();
  // 设置消息的坐标系为全局坐标系
  cloud_with_weights_msg->header.frame_id = global_frame_id_;
  // 调整粒子云消息的大小为样本集的大小
  cloud_with_weights_msg->particles.resize(set->sample_count);
  // 遍历样本集中的每个样本
  for (int i = 0; i < set->sample_count; i++) {
    // 设置粒子的 x, y, z 坐标
    cloud_with_weights_msg->particles[i].pose.position.x = set->samples[i].pose.v[0];
    cloud_with_weights_msg->particles[i].pose.position.y = set->samples[i].pose.v[1];
    cloud_with_weights_msg->particles[i].pose.position.z = 0;
    // 设置粒子的朝向（orientation），从样本的角度计算
    cloud_with_weights_msg->particles[i].pose.orientation = orientationAroundZAxis(
      set->samples[i].pose.v[2]);
    // 设置粒子的权重
    cloud_with_weights_msg->particles[i].weight = set->samples[i].weight;
  }
  // 使用发布者对象发布粒子云消息
  particle_cloud_pub_->publish(std::move(cloud_with_weights_msg));
}

bool
AmclNode::getMaxWeightHyp(
  std::vector<amcl_hyp_t> & hyps, amcl_hyp_t & max_weight_hyps,
  int & max_weight_hyp)
{
  // Read out the current hypotheses
  // 读取当前的假设（hypotheses）
  double max_weight = 0.0;
  // 为存储假设的 vector 分配空间
  hyps.resize(pf_->sets[pf_->current_set].cluster_count);
  // 遍历每个假设
  for (int hyp_count = 0;
    hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
  {
    double weight;
    pf_vector_t pose_mean;
    pf_matrix_t pose_cov;
    // 获取假设的权重、平均姿态和协方差矩阵
    if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov)) {
      RCLCPP_ERROR(get_logger(), "Couldn't get stats on cluster %d", hyp_count);
      return false;
    }
    // 将权重、平均姿态和协方差矩阵保存到假设向量中
    hyps[hyp_count].weight = weight;
    hyps[hyp_count].pf_pose_mean = pose_mean;
    hyps[hyp_count].pf_pose_cov = pose_cov;
    // 找到具有最大权重的假设
    if (hyps[hyp_count].weight > max_weight) {
      max_weight = hyps[hyp_count].weight;
      max_weight_hyp = hyp_count;
    }
  }
  // 将具有最大权重的假设信息保存到 max_weight_hyps
  if (max_weight > 0.0) {
    RCLCPP_DEBUG(
      get_logger(), "Max weight pose: %.3f %.3f %.3f",
      hyps[max_weight_hyp].pf_pose_mean.v[0],
      hyps[max_weight_hyp].pf_pose_mean.v[1],
      hyps[max_weight_hyp].pf_pose_mean.v[2]);
    // 将具有最大权重的假设信息保存到 max_weight_hyps
    max_weight_hyps = hyps[max_weight_hyp];
    return true;
  }
  return false;
}

void
AmclNode::publishAmclPose(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp)
{
  // If initial pose is not known, AMCL does not know the current pose
  // 如果初始姿态未知，则AMCL无法发布姿态或更新变换
  if (!initial_pose_is_known_) {
    if (checkElapsedTime(2s, last_time_printed_msg_)) {
      RCLCPP_WARN(
        get_logger(), "AMCL cannot publish a pose or update the transform. "
        "Please set the initial pose...");
      last_time_printed_msg_ = now();
    }
    return;
  }

  // 生成包含位姿及协方差的消息对象
  auto p = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
  // Fill in the header
  // 填充消息头
  p->header.frame_id = global_frame_id_;
  p->header.stamp = laser_scan->header.stamp;
  // Copy in the pose
  // 将估计的位姿信息复制到消息中
  p->pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
  p->pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
  p->pose.pose.orientation = orientationAroundZAxis(hyps[max_weight_hyp].pf_pose_mean.v[2]);
  // Copy in the covariance, converting from 3-D to 6-D
  // 将协方差矩阵的信息复制到消息中，从3维协方差矩阵转换为6维
  pf_sample_set_t * set = pf_->sets + pf_->current_set;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      // p->covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
      p->pose.covariance[6 * i + j] = set->cov.m[i][j];
    }
  }
  p->pose.covariance[6 * 5 + 5] = set->cov.m[2][2];
  // 计算总的协方差矩阵和位姿，以检查是否有NaN值
  float temp = 0.0;
  for (auto covariance_value : p->pose.covariance) {
    temp += covariance_value;
  }
  temp += p->pose.pose.position.x + p->pose.pose.position.y;
  // 检查协方差矩阵和位姿是否有效，如果有效则发布位姿消息
  if (!std::isnan(temp)) {
    RCLCPP_DEBUG(get_logger(), "Publishing pose");
    last_published_pose_ = *p;
    first_pose_sent_ = true;
    pose_pub_->publish(std::move(p));
  } else {
    RCLCPP_WARN(
      get_logger(), "AMCL covariance or pose is NaN, likely due to an invalid "
      "configuration or faulty sensor measurements! Pose is not available!");
  }
  // 打印发布的位姿信息
  RCLCPP_DEBUG(
    get_logger(), "New pose: %6.3f %6.3f %6.3f",
    hyps[max_weight_hyp].pf_pose_mean.v[0],
    hyps[max_weight_hyp].pf_pose_mean.v[1],
    hyps[max_weight_hyp].pf_pose_mean.v[2]);
}

void
AmclNode::calculateMaptoOdomTransform(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp)
{
  // subtracting base to odom from map to base and send map to odom instead
  geometry_msgs::msg::PoseStamped odom_to_map;
  try {
    // 创建一个包含旋转的四元数，表示从base_link到地图坐标系的旋转变换
    tf2::Quaternion q;
    q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
    // 创建从base_link到地图坐标系的刚性变换
    tf2::Transform tmp_tf(q, tf2::Vector3(
        hyps[max_weight_hyp].pf_pose_mean.v[0],
        hyps[max_weight_hyp].pf_pose_mean.v[1],
        0.0));

    geometry_msgs::msg::PoseStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = base_frame_id_;
    tmp_tf_stamped.header.stamp = laser_scan->header.stamp;
    // 将刚性变换转换为消息，并进行逆变换，得到从里程计坐标系到地图坐标系的变换
    tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);
    // 使用TF库进行坐标系变换，得到从地图到里程计坐标系的变换
    tf_buffer_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
  } catch (tf2::TransformException & e) {
    // 如果变换失败，输出错误信息并返回
    RCLCPP_DEBUG(get_logger(), "Failed to subtract base to odom transform: (%s)", e.what());
    return;
  }
  // 将计算得到的地图到里程计坐标系的变换转换为tf2库的Transform类型
  tf2::impl::Converter<true, false>::convert(odom_to_map.pose, latest_tf_);
  latest_tf_valid_ = true;    // 设置变换为有效
}

void
AmclNode::sendMapToOdomTransform(const tf2::TimePoint & transform_expiration)
{
  // AMCL will update transform only when it has knowledge about robot's initial position
  if (!initial_pose_is_known_) {return;}  // AMCL只有在获取到机器人初始位置的情况下才会更新变换
  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  tmp_tf_stamped.header.frame_id = global_frame_id_;
  tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
  tmp_tf_stamped.child_frame_id = odom_frame_id_;
  // 将地图到里程计的变换逆转并转换为消息类型
  tf2::impl::Converter<false, true>::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
  // 使用tf2库的广播器来发布地图到里程计的变换
  tf_broadcaster_->sendTransform(tmp_tf_stamped);
}

// 在 AMCL 节点中创建一个激光传感器模型对象。这个对象用于计算机器人在地图上的位置似然值，以便进行自定位
nav2_amcl::Laser *
AmclNode::createLaserObject()
{
  RCLCPP_INFO(get_logger(), "createLaserObject");
  // 根据 sensor_model_type_ 的不同值创建不同的激光传感器模型对象
  if (sensor_model_type_ == "beam") {
    // 创建并返回一个 BeamModel 对象
    return new nav2_amcl::BeamModel(
      z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_,
      0.0, max_beams_, map_);
  }

  if (sensor_model_type_ == "likelihood_field_prob") {
    // 创建并返回一个 LikelihoodFieldModelProb 对象
    return new nav2_amcl::LikelihoodFieldModelProb(
      z_hit_, z_rand_, sigma_hit_,
      laser_likelihood_max_dist_, do_beamskip_, beam_skip_distance_, beam_skip_threshold_,
      beam_skip_error_threshold_, max_beams_, map_);
  }
  // 默认情况下，创建并返回一个 LikelihoodFieldModel 对象
  return new nav2_amcl::LikelihoodFieldModel(
    z_hit_, z_rand_, sigma_hit_,
    laser_likelihood_max_dist_, max_beams_, map_);
}

void
AmclNode::initParameters()
{
  double save_pose_rate;
  double tmp_tol;

  get_parameter("alpha1", alpha1_);
  get_parameter("alpha2", alpha2_);
  get_parameter("alpha3", alpha3_);
  get_parameter("alpha4", alpha4_);
  get_parameter("alpha5", alpha5_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("beam_skip_distance", beam_skip_distance_);
  get_parameter("beam_skip_error_threshold", beam_skip_error_threshold_);
  get_parameter("beam_skip_threshold", beam_skip_threshold_);
  get_parameter("do_beamskip", do_beamskip_);
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("lambda_short", lambda_short_);
  get_parameter("laser_likelihood_max_dist", laser_likelihood_max_dist_);
  get_parameter("laser_max_range", laser_max_range_);
  get_parameter("laser_min_range", laser_min_range_);
  get_parameter("laser_model_type", sensor_model_type_);
  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose.x", initial_pose_x_);
  get_parameter("initial_pose.y", initial_pose_y_);
  get_parameter("initial_pose.z", initial_pose_z_);
  get_parameter("initial_pose.yaw", initial_pose_yaw_);
  get_parameter("max_beams", max_beams_);
  get_parameter("max_particles", max_particles_);
  get_parameter("min_particles", min_particles_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("pf_err", pf_err_);
  get_parameter("pf_z", pf_z_);
  get_parameter("recovery_alpha_fast", alpha_fast_);
  get_parameter("recovery_alpha_slow", alpha_slow_);
  get_parameter("resample_interval", resample_interval_);
  get_parameter("robot_model_type", robot_model_type_);
  get_parameter("save_pose_rate", save_pose_rate);
  get_parameter("sigma_hit", sigma_hit_);
  get_parameter("tf_broadcast", tf_broadcast_);
  get_parameter("transform_tolerance", tmp_tol);
  get_parameter("update_min_a", a_thresh_);
  get_parameter("update_min_d", d_thresh_);
  get_parameter("z_hit", z_hit_);
  get_parameter("z_max", z_max_);
  get_parameter("z_rand", z_rand_);
  get_parameter("z_short", z_short_);
  get_parameter("first_map_only", first_map_only_);
  get_parameter("always_reset_initial_pose", always_reset_initial_pose_);
  get_parameter("scan_topic", scan_topic_);
  get_parameter("map_topic", map_topic_);

  save_pose_period_ = tf2::durationFromSec(1.0 / save_pose_rate);
  transform_tolerance_ = tf2::durationFromSec(tmp_tol);

  odom_frame_id_ = nav2_util::strip_leading_slash(odom_frame_id_);
  base_frame_id_ = nav2_util::strip_leading_slash(base_frame_id_);
  global_frame_id_ = nav2_util::strip_leading_slash(global_frame_id_);

  last_time_printed_msg_ = now();

  // Semantic checks
  if (laser_likelihood_max_dist_ < 0) {
    RCLCPP_WARN(
      get_logger(), "You've set laser_likelihood_max_dist to be negtive,"
      " this isn't allowed so it will be set to default value 2.0.");
    laser_likelihood_max_dist_ = 2.0;
  }
  if (max_particles_ < 0) {
    RCLCPP_WARN(
      get_logger(), "You've set max_particles to be negtive,"
      " this isn't allowed so it will be set to default value 2000.");
    max_particles_ = 2000;
  }

  if (min_particles_ < 0) {
    RCLCPP_WARN(
      get_logger(), "You've set min_particles to be negtive,"
      " this isn't allowed so it will be set to default value 500.");
    min_particles_ = 500;
  }

  if (min_particles_ > max_particles_) {
    RCLCPP_WARN(
      get_logger(), "You've set min_particles to be greater than max particles,"
      " this isn't allowed so max_particles will be set to min_particles.");
    max_particles_ = min_particles_;
  }

  if (resample_interval_ <= 0) {
    RCLCPP_WARN(
      get_logger(), "You've set resample_interval to be zero or negtive,"
      " this isn't allowed so it will be set to default value to 1.");
    resample_interval_ = 1;
  }

  if (always_reset_initial_pose_) {
    initial_pose_is_known_ = false;
  }
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
rcl_interfaces::msg::SetParametersResult
AmclNode::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  double save_pose_rate;
  double tmp_tol;

  int max_particles = max_particles_;
  int min_particles = min_particles_;

  bool reinit_pf = false;
  bool reinit_odom = false;
  bool reinit_laser = false;
  bool reinit_map = false;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "alpha1") {
        alpha1_ = parameter.as_double();
        //alpha restricted to be non-negative
        if (alpha1_ < 0.0) {
          RCLCPP_WARN(
            get_logger(), "You've set alpha1 to be negative,"
            " this isn't allowed, so the alpha1 will be set to be zero.");
          alpha1_ = 0.0;
        }
        reinit_odom = true;
      } else if (param_name == "alpha2") {
        alpha2_ = parameter.as_double();
        //alpha restricted to be non-negative
        if (alpha2_ < 0.0) {
          RCLCPP_WARN(
            get_logger(), "You've set alpha2 to be negative,"
            " this isn't allowed, so the alpha2 will be set to be zero.");
          alpha2_ = 0.0;
        }
        reinit_odom = true;
      } else if (param_name == "alpha3") {
        alpha3_ = parameter.as_double();
        //alpha restricted to be non-negative
        if (alpha3_ < 0.0) {
          RCLCPP_WARN(
            get_logger(), "You've set alpha3 to be negative,"
            " this isn't allowed, so the alpha3 will be set to be zero.");
          alpha3_ = 0.0;
        }
        reinit_odom = true;
      } else if (param_name == "alpha4") {
        alpha4_ = parameter.as_double();
        //alpha restricted to be non-negative
        if (alpha4_ < 0.0) {
          RCLCPP_WARN(
            get_logger(), "You've set alpha4 to be negative,"
            " this isn't allowed, so the alpha4 will be set to be zero.");
          alpha4_ = 0.0;
        }
        reinit_odom = true;
      } else if (param_name == "alpha5") {
        alpha5_ = parameter.as_double();
        //alpha restricted to be non-negative
        if (alpha5_ < 0.0) {
          RCLCPP_WARN(
            get_logger(), "You've set alpha5 to be negative,"
            " this isn't allowed, so the alpha5 will be set to be zero.");
          alpha5_ = 0.0;
        }
        reinit_odom = true;
      } else if (param_name == "beam_skip_distance") {
        beam_skip_distance_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "beam_skip_error_threshold") {
        beam_skip_error_threshold_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "beam_skip_threshold") {
        beam_skip_threshold_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "lambda_short") {
        lambda_short_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "laser_likelihood_max_dist") {
        laser_likelihood_max_dist_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "laser_max_range") {
        laser_max_range_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "laser_min_range") {
        laser_min_range_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "pf_err") {
        pf_err_ = parameter.as_double();
        reinit_pf = true;
      } else if (param_name == "pf_z") {
        pf_z_ = parameter.as_double();
        reinit_pf = true;
      } else if (param_name == "recovery_alpha_fast") {
        alpha_fast_ = parameter.as_double();
        reinit_pf = true;
      } else if (param_name == "recovery_alpha_slow") {
        alpha_slow_ = parameter.as_double();
        reinit_pf = true;
      } else if (param_name == "save_pose_rate") {
        save_pose_rate = parameter.as_double();
        save_pose_period_ = tf2::durationFromSec(1.0 / save_pose_rate);
      } else if (param_name == "sigma_hit") {
        sigma_hit_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "transform_tolerance") {
        tmp_tol = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(tmp_tol);
        reinit_laser = true;
      } else if (param_name == "update_min_a") {
        a_thresh_ = parameter.as_double();
      } else if (param_name == "update_min_d") {
        d_thresh_ = parameter.as_double();
      } else if (param_name == "z_hit") {
        z_hit_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "z_max") {
        z_max_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "z_rand") {
        z_rand_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "z_short") {
        z_short_ = parameter.as_double();
        reinit_laser = true;
      }
    } else if (param_type == ParameterType::PARAMETER_STRING) {
      if (param_name == "base_frame_id") {
        base_frame_id_ = parameter.as_string();
      } else if (param_name == "global_frame_id") {
        global_frame_id_ = parameter.as_string();
      } else if (param_name == "map_topic") {
        map_topic_ = parameter.as_string();
        reinit_map = true;
      } else if (param_name == "laser_model_type") {
        sensor_model_type_ = parameter.as_string();
        reinit_laser = true;
      } else if (param_name == "odom_frame_id") {
        odom_frame_id_ = parameter.as_string();
        reinit_laser = true;
      } else if (param_name == "scan_topic") {
        scan_topic_ = parameter.as_string();
        reinit_laser = true;
      } else if (param_name == "robot_model_type") {
        robot_model_type_ = parameter.as_string();
        reinit_odom = true;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == "do_beamskip") {
        do_beamskip_ = parameter.as_bool();
        reinit_laser = true;
      } else if (param_name == "tf_broadcast") {
        tf_broadcast_ = parameter.as_bool();
      } else if (param_name == "set_initial_pose") {
        set_initial_pose_ = parameter.as_bool();
      } else if (param_name == "first_map_only") {
        first_map_only_ = parameter.as_bool();
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == "max_beams") {
        max_beams_ = parameter.as_int();
        reinit_laser = true;
      } else if (param_name == "max_particles") {
        max_particles_ = parameter.as_int();
        reinit_pf = true;
      } else if (param_name == "min_particles") {
        min_particles_ = parameter.as_int();
        reinit_pf = true;
      } else if (param_name == "resample_interval") {
        resample_interval_ = parameter.as_int();
      }
    }
  }

  // Checking if the minimum particles is greater than max_particles_
  if (min_particles_ > max_particles_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "You've set min_particles to be greater than max particles,"
      " this isn't allowed.");
    // sticking to the old values
    max_particles_ = max_particles;
    min_particles_ = min_particles;
    result.successful = false;
    return result;
  }

  // Re-initialize the particle filter
  if (reinit_pf) {
    if (pf_ != NULL) {
      pf_free(pf_);
      pf_ = NULL;
    }
    initParticleFilter();
  }

  // Re-initialize the odometry
  if (reinit_odom) {
    motion_model_.reset();
    initOdometry();
  }

  // Re-initialize the lasers and it's filters
  if (reinit_laser) {
    lasers_.clear();
    lasers_update_.clear();
    frame_to_laser_.clear();
    laser_scan_connection_.disconnect();
    laser_scan_sub_.reset();

    initMessageFilters();
  }

  // Re-initialize the map
  if (reinit_map) {
    map_sub_.reset();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&AmclNode::mapReceived, this, std::placeholders::_1));
  }

  result.successful = true;
  return result;
}

void      // 处理接收到的地图消息
AmclNode::mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "AmclNode: A new map was received.");
  // 如果 first_map_only_ 参数为真，并且已经接收到了第一张地图，则直接返回，不进行处理
  if (first_map_only_ && first_map_received_) {
    return;
  }
  // 处理接收到的地图消息
  handleMapMessage(*msg);
  // 设置 first_map_received_ 标志为 true，表示已经接收到了第一张地图
  first_map_received_ = true;
}

void
AmclNode::handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  // 打印接收到的地图信息，包括地图大小和分辨率
  RCLCPP_INFO(
    get_logger(), "Received a %d X %d map @ %.3f m/pix",
    msg.info.width,
    msg.info.height,
    msg.info.resolution);
  // 检查地图消息的帧 ID 是否与全局帧 ID 一致，如果不一致，则打印警告信息
  if (msg.header.frame_id != global_frame_id_) {
    RCLCPP_WARN(
      get_logger(), "Frame_id of map received:'%s' doesn't match global_frame_id:'%s'. This could"
      " cause issues with reading published topics",
      msg.header.frame_id.c_str(),
      global_frame_id_.c_str());
  }
  freeMapDependentMemory();   // 释放之前分配的与地图相关的内存
  map_ = convertMap(msg);     // 将接收到的地图消息转换为内部地图表示，并赋值给 map_ 变量

#if NEW_UNIFORM_SAMPLING    // 如果定义了 NEW_UNIFORM_SAMPLING，创建自由空间的向量
  createFreeSpaceVector();
#endif
}

void
AmclNode::createFreeSpaceVector()
{
  // Index of free space
  // 清空自由空间索引向量
  free_space_indices.resize(0);
  // 遍历地图的每个栅格
  for (int i = 0; i < map_->size_x; i++) {
    for (int j = 0; j < map_->size_y; j++) {
      // 如果栅格的占据状态为 -1（表示自由空间），则将其坐标添加到自由空间索引向量中
      if (map_->cells[MAP_INDEX(map_, i, j)].occ_state == -1) {
        free_space_indices.push_back(std::make_pair(i, j));
      }
    }
  }
}

void
AmclNode::freeMapDependentMemory()
{
  if (map_ != NULL) {   // 如果存在地图对象
    map_free(map_);     // 释放地图内存
    map_ = NULL;
  }

  // Clear queued laser objects because they hold pointers to the existing
  // map, #5202.
  // 清空激光对象和相关数据
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();
}

// Convert an OccupancyGrid map message into the internal representation. This function
// allocates a map_t and returns it.
map_t *
AmclNode::convertMap(const nav_msgs::msg::OccupancyGrid & map_msg)
{
  map_t * map = map_alloc();    // 分配地图对象的内存

  // 设置地图尺寸和分辨率
  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  // 设置地图原点坐标
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // 分配地图单元格的内存
  map->cells =
    reinterpret_cast<map_cell_t *>(malloc(sizeof(map_cell_t) * map->size_x * map->size_y));

  // Convert to player format
  // 将占据栅格地图数据转换为地图数据
  for (int i = 0; i < map->size_x * map->size_y; i++) {
    if (map_msg.data[i] == 0) {
      map->cells[i].occ_state = -1;   // 自由空间
    } else if (map_msg.data[i] == 100) {
      map->cells[i].occ_state = +1;   // 障碍物
    } else {
      map->cells[i].occ_state = 0;    //未知
    }
  }

  return map;
}

void
AmclNode::initTransforms()
{
  // 创建并初始化变换监听器和广播器
  RCLCPP_INFO(get_logger(), "initTransforms");

  // Initialize transform listener and broadcaster
  // 创建一个缓存对象，用于存储变换信息
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface(),
    callback_group_);
  // 将计时器接口与缓存对象关联，以便创建计时器
  tf_buffer_->setCreateTimerInterface(timer_interface);
  // 创建变换监听器，传入缓存对象
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // 创建变换广播器，将 AMCL 节点作为其所有者
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
  // 初始化变量，表示尚未发送第一个变换
  sent_first_transform_ = false;
  // 初始化最新的变换为单位变换矩阵，标记为无效
  latest_tf_valid_ = false;
  latest_tf_ = tf2::Transform::getIdentity();
}

void
AmclNode::initMessageFilters()
{
  // 创建激光扫描消息的订阅者
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;
  laser_scan_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
      rclcpp_lifecycle::LifecycleNode>>(
    shared_from_this(), scan_topic_, rmw_qos_profile_sensor_data, sub_opt);
  // 创建激光扫描消息的过滤器
  laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *laser_scan_sub_, *tf_buffer_, odom_frame_id_, 10,
    get_node_logging_interface(),
    get_node_clock_interface(),
    transform_tolerance_);

  // 注册回调函数，当符合过滤条件时触发
  laser_scan_connection_ = laser_scan_filter_->registerCallback(
    std::bind(
      &AmclNode::laserReceived,
      this, std::placeholders::_1));
}

void
AmclNode::initPubSub()
{
  RCLCPP_INFO(get_logger(), "initPubSub");
  // 创建发布者，发布粒子云数据
  particle_cloud_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>(
    "particle_cloud",
    rclcpp::SensorDataQoS());
  // 创建发布者，发布估计的机器人姿态数据
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  // 创建订阅者，用于接收初始姿态信息
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&AmclNode::initialPoseReceived, this, std::placeholders::_1));
  // 创建订阅者，用于接收地图数据
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&AmclNode::mapReceived, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Subscribed to map topic.");
}

void
AmclNode::initServices()
{
  // 重新初始化全局定位
  global_loc_srv_ = create_service<std_srvs::srv::Empty>(
    "reinitialize_global_localization",
    std::bind(&AmclNode::globalLocalizationCallback, this, _1, _2, _3));
  // 请求无运动更新
  nomotion_update_srv_ = create_service<std_srvs::srv::Empty>(
    "request_nomotion_update",
    std::bind(&AmclNode::nomotionUpdateCallback, this, _1, _2, _3));
}

void
AmclNode::initOdometry()
{
  // TODO(mjeronimo): We should handle persistance of the last known pose of the robot. We could
  // then read that pose here and initialize using that.
  // TODO: 处理机器人上一次已知的姿态的持久性。然后我们可以在此处读取该姿态并使用它来进行初始化

  // When pausing and resuming, remember the last robot pose so we don't start at 0:0 again
  // 在暂停和恢复时，记住机器人的上一次姿态，以便我们不会从 0:0 开始
  init_pose_[0] = last_published_pose_.pose.pose.position.x;
  init_pose_[1] = last_published_pose_.pose.pose.position.y;
  init_pose_[2] = tf2::getYaw(last_published_pose_.pose.pose.orientation);

  if (!initial_pose_is_known_) {
    init_cov_[0] = 0.5 * 0.5;
    init_cov_[1] = 0.5 * 0.5;
    init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);
  } else {
    init_cov_[0] = last_published_pose_.pose.covariance[0];
    init_cov_[1] = last_published_pose_.pose.covariance[7];
    init_cov_[2] = last_published_pose_.pose.covariance[35];
  }
  // 使用插件加载器创建并初始化运动模型
  motion_model_ = plugin_loader_.createSharedInstance(robot_model_type_);
  motion_model_->initialize(alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
  // 初始化保存最新里程计姿态的变量
  latest_odom_pose_ = geometry_msgs::msg::PoseStamped();
}

// 首先使用 pf_alloc 函数创建了一个粒子滤波器 pf_。接着，设置了一些滤波器参数，
// 如最小和最大粒子数、减速和加速参数等。然后，使用初始化的机器人姿态和协方差矩阵，
// 调用 pf_init 函数来初始化粒子滤波器。接下来，设置了一些额外的变量，
// 如 pf_init_ 标志、resample_count_ 计数器，以及清空了 pf_odom_pose_ 变量
void
AmclNode::initParticleFilter()
{
  // Create the particle filter
  // 创建粒子滤波器
  pf_ = pf_alloc(
    min_particles_, max_particles_, alpha_slow_, alpha_fast_,
    (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
    reinterpret_cast<void *>(map_));
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter
  // 初始化滤波器
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];

  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov_[0];
  pf_init_pose_cov.m[1][1] = init_cov_[1];
  pf_init_pose_cov.m[2][2] = init_cov_[2];

  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);

  pf_init_ = false;
  resample_count_ = 0;
  memset(&pf_odom_pose_, 0, sizeof(pf_odom_pose_));
}

void
AmclNode::initLaserScan()
{
  scan_error_count_ = 0;  //用于计算激光扫描的错误次数
  last_laser_received_ts_ = rclcpp::Time(0);  //用于存储上次接收到的激光扫描的时间戳
}

}  // namespace nav2_amcl

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
// 通过class_loader注册组件。
// 这充当一种入口点，使组件在其库被加载到运行中的进程中时可以被发现。
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_amcl::AmclNode)
