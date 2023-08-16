/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
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
/**************************************************************************
 * Desc: Simple particle filter for localization.
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf.c 6345 2008-04-17 01:36:39Z gerkey $
 *************************************************************************/

#include <float.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_pdf.hpp"
#include "nav2_amcl/pf/pf_kdtree.hpp"

#include "nav2_amcl/portable_utils.hpp"


// Compute the required number of samples, given that there are k bins
// with samples in them.
static int pf_resample_limit(pf_t * pf, int k);


// Create a new filter
// 用于分配并初始化一个新的粒子滤波器对象。它接受多个参数，
// 包括最小样本数量、最大样本数量、alpha_slow、alpha_fast等
pf_t * pf_alloc(
  int min_samples, int max_samples,
  double alpha_slow, double alpha_fast,
  pf_init_model_fn_t random_pose_fn, void * random_pose_data)
{
  int i, j;
  pf_t * pf;
  pf_sample_set_t * set;
  pf_sample_t * sample;
  // 使用当前时间来初始化随机数生成器，以便在粒子滤波器的过程中生成随机数
  srand48(time(NULL));
  // 通过 calloc 分配了一个大小为 sizeof(pf_t) 的内存块，并将其初始化为零。这个内存块表示整个粒子滤波器对象
  pf = calloc(1, sizeof(pf_t));
  // 将传递的函数指针 random_pose_fn 和数据指针 random_pose_data 分别赋值给
  // pf 结构中的对应成员。这些指针将用于在初始化粒子时生成随机的初始姿态
  pf->random_pose_fn = random_pose_fn;
  pf->random_pose_data = random_pose_data;

  pf->min_samples = min_samples;
  pf->max_samples = max_samples;

  // Control parameters for the population size calculation.  [err] is
  // the max error between the true distribution and the estimated
  // distribution.  [z] is the upper standard normal quantile for (1 -
  // p), where p is the probability that the error on the estimated
  // distrubition will be less than [err].
  // 设置了粒子滤波器对象的一些参数，如 pop_err 是用于估计误差的参数，
  // pop_z 是用于估计高斯分布的参数，dist_threshold 是用于确定粒子是否需要被重新采样的距离阈值
  pf->pop_err = 0.01;
  pf->pop_z = 3;
  pf->dist_threshold = 0.5;
  // 这表示当前使用的样本集编号
  pf->current_set = 0;
  // 初始化了两个样本集。循环迭代两次，对每个样本集进行初始化。对于每个样本集，
  // 将 sample_count 设置为最大样本数量 max_samples，然后使用 calloc 分配了
  // 一个数组来存储样本信息，每个样本类型是 pf_sample_t。每个样本都被初始化为位姿为0的状态，
  // 并且初始权重为 1.0 / max_samples
  for (j = 0; j < 2; j++) {
    set = pf->sets + j;

    set->sample_count = max_samples;
    set->samples = calloc(max_samples, sizeof(pf_sample_t));

    for (i = 0; i < set->sample_count; i++) {
      sample = set->samples + i;
      sample->pose.v[0] = 0.0;
      sample->pose.v[1] = 0.0;
      sample->pose.v[2] = 0.0;
      sample->weight = 1.0 / max_samples;
    }

    // HACK: is 3 times max_samples enough?
    // 为每个样本集分配了内存以存储聚类信息。kdtree 用于快速搜索附近的样本，
    // clusters 用于存储聚类信息。mean 和 cov 分别初始化为零，用于存储每个样本集的均值和协方差
    set->kdtree = pf_kdtree_alloc(3 * max_samples);

    set->cluster_count = 0;
    set->cluster_max_count = max_samples;
    set->clusters = calloc(set->cluster_max_count, sizeof(pf_cluster_t));

    set->mean = pf_vector_zero();
    set->cov = pf_matrix_zero();
  }
  // 用于平滑更新权重的过程
  pf->w_slow = 0.0;
  pf->w_fast = 0.0;
  // 设置为传入的参数值，这些参数用于权重更新的过程，以不同的速率进行权重的调整
  pf->alpha_slow = alpha_slow;
  pf->alpha_fast = alpha_fast;

  // set converged to 0
  // 表示粒子滤波器还没有收敛
  pf_init_converged(pf);

  return pf;
}

// Free an existing filter
// 实现了粒子滤波器对象的内存释放。它遍历了粒子滤波器对象的两个 pf_sample_set_t 集合，
// 分别释放了每个集合中的内存资源，包括 clusters、kdtree 和 samples。
// 最后，释放了整个粒子滤波器对象的内存资源。
void pf_free(pf_t * pf)
{
  int i;

  for (i = 0; i < 2; i++) {
    free(pf->sets[i].clusters);
    pf_kdtree_free(pf->sets[i].kdtree);
    free(pf->sets[i].samples);
  }
  free(pf);
}

// Initialize the filter using a guassian
// 初始化粒子滤波器对象中的粒子集合，并创建适应性采样所需的 kd 树。
// 它还为粒子集合中的每个粒子样本创建了一个高斯分布的概率密度函数
void pf_init(pf_t * pf, pf_vector_t mean, pf_matrix_t cov)
{
  int i;
  // 获取当前的粒子集合 set，它是 pf->sets 数组中的一个元素，表示当前要处理的样本集合
  pf_sample_set_t * set;
  pf_sample_t * sample;
  pf_pdf_gaussian_t * pdf;

  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  // 清空当前集合的 kd 树，以便重新建立适应性采样所需的数据结构
  pf_kdtree_clear(set->kdtree);
  // 将当前粒子集合的样本数量设置为最大样本数量
  set->sample_count = pf->max_samples;
  // 创建一个高斯分布的概率密度函数 pdf，用于表示每个粒子样本的初始状态。这里传入了初始均值 mean 和初始协方差矩阵 cov
  pdf = pf_pdf_gaussian_alloc(mean, cov);

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++) {
    // 获取当前粒子样本 sample，即当前迭代的样本
    sample = set->samples + i;
    // 将当前粒子样本的权重初始化为均匀分布的初始权重，即 1.0 / pf->max_samples。这表示在初始状态下，每个粒子样本的权重是相等的
    sample->weight = 1.0 / pf->max_samples;
    // 使用 pf_pdf_gaussian_sample 函数从之前创建的高斯分布的概率密度函数 pdf 中采样，得到一个新的粒子姿态（位置和方向）
    sample->pose = pf_pdf_gaussian_sample(pdf);

    // Add sample to histogram
    // 将采样的粒子姿态和权重添加到 kd 树中，以便后续的适应性采样
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }
  // 将粒子滤波器的 w_slow 和 w_fast 参数设置为 0，这是为了将粒子滤波器的收敛参数初始化为初始值
  pf->w_slow = pf->w_fast = 0.0;

  pf_pdf_gaussian_free(pdf);

  // Re-compute cluster statistics
  // 重新计算当前粒子集合的聚类统计信息，即调用 pf_cluster_stats 函数计算聚类数、聚类均值和协方差等统计信息
  pf_cluster_stats(pf, set);

  // set converged to 0
  // 将粒子滤波器的收敛状态标志 converged 重新设置为 0，以表示初始化后尚未达到收敛状态
  pf_init_converged(pf);
}


// Initialize the filter using some model
void pf_init_model(pf_t * pf, pf_init_model_fn_t init_fn, void * init_data)
{
  int i;
  pf_sample_set_t * set;
  pf_sample_t * sample;

  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  // 清除当前粒子集合中的 kd 树
  pf_kdtree_clear(set->kdtree);
  // 将当前粒子集合中的样本数量设置为最大样本数
  set->sample_count = pf->max_samples;

  // Compute the new sample poses
  // 使用提供的初始化函数 init_fn 来计算新的样本姿态。循环遍历每个样本，
  // 将样本权重初始化为均匀分布的初始权重，即 1.0 / pf->max_samples。
  // 然后，调用初始化函数 init_fn 并传入 init_data 作为参数，以获取一个新的粒子姿态。
  // 这里的 init_fn 应该是一个用户定义的函数，用于生成粒子的初始姿态
  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->pose = (*init_fn)(init_data);

    // Add sample to histogram
    // 将采样的粒子姿态和权重添加到 kd 树中，以便后续的适应性采样
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }
  // 将粒子滤波器的 w_slow 和 w_fast 参数设置为 0，将收敛参数初始化为初始值
  pf->w_slow = pf->w_fast = 0.0;

  // Re-compute cluster statistics
  // 重新计算当前粒子集合的聚类统计信息，即调用 pf_cluster_stats 函数计算聚类数、聚类均值和协方差等统计信息
  pf_cluster_stats(pf, set);

  // set converged to 0
  // 将粒子滤波器的收敛状态标志 converged 重新设置为 0，表示初始化后尚未达到收敛状态
  pf_init_converged(pf);
}

// 将粒子滤波器的收敛状态标志重置为未收敛状态
void pf_init_converged(pf_t * pf)
{
  pf_sample_set_t * set;
  set = pf->sets + pf->current_set;   // 获取当前粒子集合
  set->converged = 0;                 // 收敛状态标志 converged 设置为 0，表示未收敛状态
  pf->converged = 0;  // 粒子滤波器的总体收敛状态标志 pf->converged 也设置为 0，表示整个滤波器未收敛状态
}

// 用于判断粒子滤波器是否已经收敛的函数。它的主要思想是基于粒子的位置判断滤波器的收敛状态
int pf_update_converged(pf_t * pf)
{
  int i;
  // 首先，定义一个指向当前样本集的指针 set，并初始化两个变量 mean_x 和 mean_y 为 0，用于计算粒子的平均位置
  pf_sample_set_t * set;
  pf_sample_t * sample;

  set = pf->sets + pf->current_set;
  double mean_x = 0, mean_y = 0;
  // 遍历当前样本集中的每个粒子，将每个粒子的 x 和 y 坐标累加到 mean_x 和 mean_y 中
  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;

    mean_x += sample->pose.v[0];
    mean_y += sample->pose.v[1];
  }
  // 计算所有粒子的平均位置，即将 mean_x 和 mean_y 分别除以样本集中的粒子数，得到平均位置
  mean_x /= set->sample_count;
  mean_y /= set->sample_count;
  // 再次遍历样本集中的每个粒子，检查粒子的 x 和 y 坐标是否与平均位置之间的距离超过了设定的阈值 pf->dist_threshold
  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    // 如果有任何一个粒子的位置超过了阈值，说明粒子分布还没有足够集中在平均位置附近，
    // 此时将 set->converged 和 pf->converged 设置为 0，表示滤波器未收敛，然后返回 0
    if (fabs(sample->pose.v[0] - mean_x) > pf->dist_threshold ||
      fabs(sample->pose.v[1] - mean_y) > pf->dist_threshold)
    {
      set->converged = 0;
      pf->converged = 0;
      return 0;
    }
  }
  // 如果所有粒子的位置都在阈值范围内，说明粒子分布已经足够集中在平均位置附近，
  // 此时将 set->converged 和 pf->converged 设置为 1，表示滤波器已收敛，然后返回 1
  set->converged = 1;
  pf->converged = 1;
  return 1;
}

// Update the filter with some new action
// void pf_update_action(pf_t * pf, pf_action_model_fn_t action_fn, void * action_data)
// {
//   pf_sample_set_t * set;

//   set = pf->sets + pf->current_set;

//   (*action_fn)(action_data, set);
// }

// Update the filter with some new sensor observation
void pf_update_sensor(pf_t * pf, pf_sensor_model_fn_t sensor_fn, void * sensor_data)
{
  int i;
  pf_sample_set_t * set;
  pf_sample_t * sample;
  double total;
  // 定义一个指向当前样本集的指针 set
  set = pf->sets + pf->current_set;

  // Compute the sample weights
  // 调用传感器模型函数 sensor_fn，该函数用于计算传感器模型并返回所有粒子的观测总权重 total。
  // 传感器模型函数的参数包括传感器数据和当前样本集
  total = (*sensor_fn)(sensor_data, set);
  // 如果观测总权重 total 大于 0.0，说明传感器观测有效
  if (total > 0.0) {
    // Normalize weights
    double w_avg = 0.0;

    // 通过循环遍历每个粒子，将粒子的权重标准化为概率，并计算所有粒子权重的平均值 w_avg。
    // 然后，使用运行平均值的方法来更新慢速和快速权重，以跟踪粒子权重的变化情况。
    // 这些权重的变化情况将用于判断滤波器是否应该进行重采样。如果传感器提供了有效的观测数据，
    // 那么通过更新粒子权重来反映观测结果。如果传感器没有有效的观测数据，那么将所有粒子的权重设置为均匀分布的概率
    for (i = 0; i < set->sample_count; i++) {
      // 通过循环遍历每个粒子，使用指针 sample 指向当前粒子
      sample = set->samples + i;
      // 计算粒子权重的平均值 w_avg，用于后续计算运行平均权重
      w_avg += sample->weight;
      sample->weight /= total;
    }
    // Update running averages of likelihood of samples (Prob Rob p258)
    // 将每个粒子的权重除以传感器观测总权重 total，从而将权重标准化为概率。这样，所有粒子的权重之和将等于 1.0
    w_avg /= set->sample_count;
    // 判断 pf->w_slow 是否为 0.0，如果是，将 pf->w_slow 设置为 w_avg
    if (pf->w_slow == 0.0) {
      pf->w_slow = w_avg;
    } else {  // 否则，使用公式 pf->w_slow += pf->alpha_slow * (w_avg - pf->w_slow) 来更新慢速权重的运行平均值
      pf->w_slow += pf->alpha_slow * (w_avg - pf->w_slow); // pf->alpha_slow 是一个控制慢速权重更新速率的参数
    }
    if (pf->w_fast == 0.0) {  // 判断 pf->w_fast 是否为 0.0，如果是，将 pf->w_fast 设置为 w_avg 
      pf->w_fast = w_avg;
    } else {  // 否则，使用公式 pf->w_fast += pf->alpha_fast * (w_avg - pf->w_fast) 来更新快速权重的运行平均值
      pf->w_fast += pf->alpha_fast * (w_avg - pf->w_fast);
    }
  } else {
    // Handle zero total
    // 如果观测总权重 total 小于等于 0.0，表示传感器没有提供有效的观测数据
    for (i = 0; i < set->sample_count; i++) {
      sample = set->samples + i;
      // 对每个粒子进行遍历，将每个粒子的权重设置为均匀分布的概率
      sample->weight = 1.0 / set->sample_count;
    }
  }
}


// Resample the distribution
void pf_update_resample(pf_t * pf)
{
  int i;
  double total;
  pf_sample_set_t * set_a, * set_b;
  pf_sample_t * sample_a, * sample_b;

  // double r,c,U;
  // int m;
  // double count_inv;
  double * c;

  double w_diff;
  // 获取当前的粒子样本集 set_a，和下一个要更新的粒子样本集 set_b。在这种实现中，每次迭代交替使用两个粒子样本集
  set_a = pf->sets + pf->current_set;
  set_b = pf->sets + (pf->current_set + 1) % 2;

  // Build up cumulative probability table for resampling.
  // TODO(?): Replace this with a more efficient procedure
  // (e.g., http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)
  
  // 分配一个数组 c，用于存储累计权重的和。数组 c 的长度为当前粒子样本集的大小加 1
  // 数组 c 中的元素用于确定每个新粒子的生成位置
  c = (double *)malloc(sizeof(double) * (set_a->sample_count + 1));
  c[0] = 0.0;
  // 使用循环遍历当前粒子样本集中的每个粒子，计算并保存累计权重的和到数组 c 中。这样，c[i + 1] 存储了前 i + 1 个粒子的累计权重之和
  for (i = 0; i < set_a->sample_count; i++) {
    c[i + 1] = c[i] + set_a->samples[i].weight;
  }

  // Create the kd tree for adaptive sampling
  // 清除下一个要更新的粒子样本集 set_b 中的 kd 树，为后续重采样做准备
  pf_kdtree_clear(set_b->kdtree);

  // Draw samples from set a to create set b.
  // 初始化变量 total，用于存储新的粒子样本集 set_b 的总权重。还将初始化 set_b 中的粒子数量为 0，表示尚未生成新的粒子
  total = 0;
  set_b->sample_count = 0;
  // 计算变量 w_diff，该变量用于根据运行平均权重来调整采样过程。它表示当前时刻平均权重变化的比例，
  // 即快速平均权重相对于慢速平均权重的差异。如果 w_diff 小于 0.0，则将其设为 0.0，确保不会出现负值
  w_diff = 1.0 - pf->w_fast / pf->w_slow;
  if (w_diff < 0.0) {
    w_diff = 0.0;
  }
  // printf("w_diff: %9.6f\n", w_diff);

  // Can't (easily) combine low-variance sampler with KLD adaptive
  // sampling, so we'll take the more traditional route.
  /*
  // Low-variance resampler, taken from Probabilistic Robotics, p110
  count_inv = 1.0/set_a->sample_count;
  r = drand48() * count_inv;
  c = set_a->samples[0].weight;
  i = 0;
  m = 0;
  */
  // 重采样的核心部分，它在循环中从当前的粒子样本集 set_a 中抽取新的粒子样本，
  // 并将其添加到下一个要更新的粒子样本集 set_b 中，直到达到最大粒子数为止
  while (set_b->sample_count < pf->max_samples) {
    // 通过 set_b->sample_count++ 获取要填充的下一个粒子的索引，并获取指向该索引的 sample_b 指针
    sample_b = set_b->samples + set_b->sample_count++;
    // 使用 drand48() 生成一个随机数，如果这个随机数小于 w_diff，则以一定的概率从 random_pose_fn 函数生成一个随机的粒子姿态 sample_b->pose
    if (drand48() < w_diff) {
      sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);
    } else {
      // Can't (easily) combine low-variance sampler with KLD adaptive
      // sampling, so we'll take the more traditional route.
      /*
      // Low-variance resampler, taken from Probabilistic Robotics, p110
      U = r + m * count_inv;
      while(U>c)
      {
        i++;
        // Handle wrap-around by resetting counters and picking a new random
        // number
        if(i >= set_a->sample_count)
        {
          r = drand48() * count_inv;
          c = set_a->samples[0].weight;
          i = 0;
          m = 0;
          U = r + m * count_inv;
          continue;
        }
        c += set_a->samples[i].weight;
      }
      m++;
      */

      // Naive discrete event sampler
      // 如果随机数大于等于 w_diff，则生成一个介于 0 和 1 之间的随机数 r，然后使用
      // 累计分布函数 c 来确定从哪个样本中抽取新的粒子。循环遍历 set_a 中的样本，
      // 找到满足条件 (c[i] <= r) && (r < c[i + 1]) 的 i 值
      double r;
      r = drand48();
      for (i = 0; i < set_a->sample_count; i++) {
        if ((c[i] <= r) && (r < c[i + 1])) {
          break;
        }
      }
      assert(i < set_a->sample_count);
      // 从 set_a 的 samples 数组中获取 sample_a
      sample_a = set_a->samples + i;

      // 确保 sample_a 的权重大于 0，然后将其姿态赋值给 sample_b->pose
      assert(sample_a->weight > 0);

      // Add sample to list
      sample_b->pose = sample_a->pose;
    }

    sample_b->weight = 1.0;   // 设置新粒子的权重为 1.0，因为在重采样过程中所有的粒子都是平等的
    total += sample_b->weight;  // 更新 total，将新粒子的权重添加到总权重中

    // Add sample to histogram
    // 将新粒子的姿态和权重添加到 set_b 的 KD 树中，以便后续的重采样过程能够使用 KD 树来进行高效的抽样操作
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);

    // See if we have enough samples yet
    // 检查是否已经达到了需要的抽样数量。调用 pf_resample_limit(pf, set_b->kdtree->leaf_count) 
    // 计算当前抽样数所需要的最小抽样数。如果已经达到了所需的抽样数量，就跳出循环
    if (set_b->sample_count > pf_resample_limit(pf, set_b->kdtree->leaf_count)) {
      break;
    }
  }

  // Reset averages, to avoid spiraling off into complete randomness.
  // 如果 w_diff 大于 0.0，表示已经进行了重采样，为了避免权重过于随机，将 pf->w_slow 和 pf->w_fast 重置为 0.0
  if (w_diff > 0.0) {
    pf->w_slow = pf->w_fast = 0.0;
  }

  // fprintf(stderr, "\n\n");

  // Normalize weights
  // 对新粒子集合 set_b 进行权重归一化，以确保所有粒子的权重总和为 1.0
  for (i = 0; i < set_b->sample_count; i++) {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }

  // Re-compute cluster statistics
  // 重新计算新粒子集合的聚类统计信息，以便在下一次更新时使用
  pf_cluster_stats(pf, set_b);

  // Use the newly created sample set
  // 切换到使用新生成的粒子集合，将当前使用的粒子集合索引 pf->current_set 切换到下一个集合，即 (pf->current_set + 1) % 2
  pf->current_set = (pf->current_set + 1) % 2;
  // 调用 pf_update_converged(pf) 更新收敛状态，检查粒子滤波器是否已经收敛
  pf_update_converged(pf);

  free(c);    // 释放之前分配的数组 c，这是用于计算新粒子权重的累积数组
}


// Compute the required number of samples, given that there are k bins
// with samples in them.  This is taken directly from Fox et al.
// 用于计算在进行重采样时需要生成多少新粒子，以确保生成足够多的新粒子以保持滤波器的有效性和精度
int pf_resample_limit(pf_t * pf, int k)
{
  double a, b, c, x;
  int n;
  // 如果粒子数量 k 小于等于 1，直接返回滤波器的最大粒子数量 pf->max_samples
  if (k <= 1) {
    return pf->max_samples;
  }
  // 计算一些系数 a、b、c 和 x，这些系数将用于估计所需的新粒子数量。其中，a、b、c 是系数，x 是用于计算新粒子数量的中间变量
  a = 1;
  b = 2 / (9 * ((double) k - 1));
  c = sqrt(2 / (9 * ((double) k - 1))) * pf->pop_z;
  // 使用系数 x，根据一定的公式估计生成新粒子的数量 n
  x = a - b + c;

  n = (int) ceil((k - 1) / (2 * pf->pop_err) * x * x * x);
  // 如果估计的新粒子数量 n 小于 pf->min_samples，则返回 pf->min_samples，以确保粒子数量不会低于设定的最小值
  if (n < pf->min_samples) {
    return pf->min_samples;
  }
  // 如果估计的新粒子数量 n 大于 pf->max_samples，则返回 pf->max_samples，以确保不会超过设定的最大值
  if (n > pf->max_samples) {
    return pf->max_samples;
  }
  // 如果估计的新粒子数量 n 在最小值和最大值之间，直接返回估计值 n
  return n;
}


// Re-compute the cluster statistics for a sample set
// 计算粒子集合中样本的聚类统计信息。在粒子滤波器中，
// 聚类可以用于合并相似的粒子，从而降低粒子数量并提高滤波器的计算效率
void pf_cluster_stats(pf_t * pf, pf_sample_set_t * set)
{
  // pf_t * pf 和 pf_sample_set_t * set 是函数的输入参数，分别表示粒子滤波器和粒子集合
  (void)pf;
  int i, j, k, cidx;
  pf_sample_t * sample;
  pf_cluster_t * cluster;

  // Workspace
  // 初始化一些变量和数组，用于计算聚类统计信息。其中，m[4] 和 c[2][2] 是用于计算聚类中心和协方差矩阵的临时数组
  double m[4], c[2][2];
  size_t count;
  double weight;

  // Cluster the samples
  // pf_kdtree_cluster(set->kdtree) 用于执行基于 k-d 树的聚类操作，将粒子样本进行聚类，将相似的粒子归为一组
  pf_kdtree_cluster(set->kdtree);

  // Initialize cluster stats
  // set->cluster_count 初始化为 0，表示当前粒子集合中的聚类数量
  set->cluster_count = 0;

  for (i = 0; i < set->cluster_max_count; i++) {
    cluster = set->clusters + i;
    cluster->count = 0;               // 将当前聚类中的样本数量初始化为0
    cluster->weight = 0;              // 将当前聚类的权重总和初始化为0
    cluster->mean = pf_vector_zero(); // 将当前聚类的均值向量初始化为零向量
    cluster->cov = pf_matrix_zero();  // 将当前聚类的协方差矩阵初始化为零矩阵

    // 循环遍历 cluster->m 数组和 cluster->c 数组，并将其各个元素初始化为0。
    // 这些数组将用于计算聚类的一些统计信息，例如均值和协方差
    for (j = 0; j < 4; j++) {
      cluster->m[j] = 0.0;
    }
    for (j = 0; j < 2; j++) {
      for (k = 0; k < 2; k++) {
        cluster->c[j][k] = 0.0;
      }
    }
  }

  // Initialize overall filter stats
  count = 0;                    // 将样本总数初始化为0，将用于统计整个样本集合的样本数量
  weight = 0.0;                 // 将样本集合的权重总和初始化为0，用于计算加权均值
  set->mean = pf_vector_zero(); // 将样本集合的均值向量初始化为零向量，用于计算加权均值
  set->cov = pf_matrix_zero();  // 将样本集合的协方差矩阵初始化为零矩阵，用于计算加权协方差
  // 循环遍历数组 m 和二维数组 c，将其各个元素初始化为0。这些数组将用于计算样本集合的加权均值和加权协方差
  for (j = 0; j < 4; j++) {
    m[j] = 0.0;
  }
  for (j = 0; j < 2; j++) {
    for (k = 0; k < 2; k++) {
      c[j][k] = 0.0;
    }
  }

  // Compute cluster stats
  for (i = 0; i < set->sample_count; i++) {
    // 获取当前迭代的样本
    sample = set->samples + i;

    // printf("%d %f %f %f\n", i, sample->pose.v[0], sample->pose.v[1], sample->pose.v[2]);

    // Get the cluster label for this sample
    // 获取当前样本所属的聚类标签（cidx），即确定样本在KD树中所属的聚类
    cidx = pf_kdtree_get_cluster(set->kdtree, sample->pose);
    // 如果 cidx 的值超过了聚类数组的最大索引，忽略该样本
    assert(cidx >= 0);
    if (cidx >= set->cluster_max_count) {
      continue;
    }
    // 如果 cidx + 1 大于当前聚类数量，将聚类数量更新为 cidx + 1
    if (cidx + 1 > set->cluster_count) {
      set->cluster_count = cidx + 1;
    }
    // 更新聚类的样本数和权重。同时，更新整个样本集合的样本数和权重
    cluster = set->clusters + cidx;

    cluster->count += 1;
    cluster->weight += sample->weight;

    count += 1;
    weight += sample->weight;

    // Compute mean
    // 计算聚类的加权均值。对于每个样本，将其权重乘以对应的位置坐标和角度（通过正弦和余弦函数计算），
    // 并将结果累加到聚类的 m 数组和整个样本集合的 m 数组中
    cluster->m[0] += sample->weight * sample->pose.v[0];
    cluster->m[1] += sample->weight * sample->pose.v[1];
    cluster->m[2] += sample->weight * cos(sample->pose.v[2]);
    cluster->m[3] += sample->weight * sin(sample->pose.v[2]);

    m[0] += sample->weight * sample->pose.v[0];
    m[1] += sample->weight * sample->pose.v[1];
    m[2] += sample->weight * cos(sample->pose.v[2]);
    m[3] += sample->weight * sin(sample->pose.v[2]);

    // Compute covariance in linear components
    // 计算聚类的加权协方差。对于每个样本，将其权重乘以对应的位置坐标，
    // 并将结果的外积（乘积）累加到聚类的协方差矩阵 c 中，同时将结果累加到整个样本集合的协方差矩阵 c 中
    for (j = 0; j < 2; j++) {
      for (k = 0; k < 2; k++) {
        cluster->c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
        c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
      }
    }
  }

  // 对每个聚类进行遍历，根据之前计算的统计信息计算聚类的均值、协方差和角度信息，然后更新聚类的状态
  // Normalize
  for (i = 0; i < set->cluster_count; i++) {
    // 获取当前迭代的聚类
    cluster = set->clusters + i;
    // 计算聚类的加权均值。使用之前累积的 cluster->m 数组和权重，计算聚类的平均位置（x、y坐标）以及平均角度，将结果保存在聚类的 mean 向量中
    cluster->mean.v[0] = cluster->m[0] / cluster->weight;
    cluster->mean.v[1] = cluster->m[1] / cluster->weight;
    cluster->mean.v[2] = atan2(cluster->m[3], cluster->m[2]);
    // 初始化聚类的协方差矩阵为零矩阵
    cluster->cov = pf_matrix_zero();

    // Covariance in linear components
    // 计算聚类的加权协方差。使用之前累积的 cluster->c 数组和权重，计算聚类的协方差矩阵。
    // 对于每个元素，将 cluster->c 中的对应元素值除以聚类的权重，然后减去对应均值的乘积，得到协方差矩阵的元素值
    for (j = 0; j < 2; j++) {
      for (k = 0; k < 2; k++) {
        cluster->cov.m[j][k] = cluster->c[j][k] / cluster->weight -
          cluster->mean.v[j] * cluster->mean.v[k];
      }
    }

    // Covariance in angular components; I think this is the correct
    // formula for circular statistics.
    // 计算聚类的角度信息。使用聚类的累积 cluster->m 数组中的值计算角度信息，
    // 将其转换为对应的角度值，并根据一定的公式计算聚类的角度协方差
    cluster->cov.m[2][2] = -2 * log(
      sqrt(
        cluster->m[2] * cluster->m[2] +
        cluster->m[3] * cluster->m[3]));

    // printf("cluster %d %d %f (%f %f %f)\n", i, cluster->count, cluster->weight,
    // cluster->mean.v[0], cluster->mean.v[1], cluster->mean.v[2]);
    // pf_matrix_fprintf(cluster->cov, stdout, "%e");
  }

  // Compute overall filter stats
  // 计算粒子集合的加权均值。使用之前累积的 m 数组和总权重，计算整个粒子集合的平均位置
  // （x、y坐标）以及平均角度，将结果保存在粒子集合的 mean 向量中
  // 计算并设置样本集合的加权均值的 x 坐标，即样本集合在 x 轴上的平均位置
  set->mean.v[0] = m[0] / weight;
  // 计算并设置样本集合的加权均值的 y 坐标，即样本集合在 y 轴上的平均位置
  set->mean.v[1] = m[1] / weight;
  // 计算并设置样本集合的加权均值的角度，即样本集合的平均角度。这里使用 atan2 函数来计算平均角度
  set->mean.v[2] = atan2(m[3], m[2]);

  // Covariance in linear components
  // 遍历协方差矩阵的所有元素
  for (j = 0; j < 2; j++) {
    for (k = 0; k < 2; k++) {
      set->cov.m[j][k] = c[j][k] / weight - set->mean.v[j] * set->mean.v[k];
    }
  }

  // Covariance in angular components; I think this is the correct
  // formula for circular statistics.
  // 这一行代码设置样本集合协方差矩阵的第三个对角元素（角度的方差）。
  // 它计算为 -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]))，其中 m[2] 和 m[3] 是之前累积的值，
  // 这部分计算与角度统计特性有关
  set->cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));
}


// Compute the CEP statistics (mean and variance).
// void pf_get_cep_stats(pf_t * pf, pf_vector_t * mean, double * var)
// {
//   int i;
//   double mn, mx, my, mrr;
//   pf_sample_set_t * set;
//   pf_sample_t * sample;

//   set = pf->sets + pf->current_set;

//   mn = 0.0;
//   mx = 0.0;
//   my = 0.0;
//   mrr = 0.0;

//   for (i = 0; i < set->sample_count; i++) {
//     sample = set->samples + i;

//     mn += sample->weight;
//     mx += sample->weight * sample->pose.v[0];
//     my += sample->weight * sample->pose.v[1];
//     mrr += sample->weight * sample->pose.v[0] * sample->pose.v[0];
//     mrr += sample->weight * sample->pose.v[1] * sample->pose.v[1];
//   }

//   mean->v[0] = mx / mn;
//   mean->v[1] = my / mn;
//   mean->v[2] = 0.0;

//   *var = mrr / mn - (mx * mx / (mn * mn) + my * my / (mn * mn));
// }


// Get the statistics for a particular cluster.
// 表明这是一个返回整数值的函数，该整数值可以指示获取统计信息的成功与否

// pf_t * pf：粒子滤波器的指针
// int clabel：要获取统计信息的集群标签（索引）
// double * weight：用于存储集群的权重（即样本权重之和）
// pf_vector_t * mean：用于存储集群的平均位置
// pf_matrix_t * cov：用于存储集群的协方差矩阵
int pf_get_cluster_stats(
  pf_t * pf, int clabel, double * weight, 
  pf_vector_t * mean, pf_matrix_t * cov)
{
  // 声明了指向样本集合和集群的指针，以便获取相应的信息
  pf_sample_set_t * set;
  pf_cluster_t * cluster;
  // 从粒子滤波器中获取当前样本集合的指针，以便从中获取集群信息
  set = pf->sets + pf->current_set;
  // 检查所请求的集群标签是否有效。如果请求的标签大于等于当前样本集合的集群数量，那么说明请求无效，函数将返回 0，表示获取失败
  if (clabel >= set->cluster_count) {
    return 0;
  }
  // 如果请求的集群标签有效，这一行代码将设置 cluster 指针指向所请求的集群
  cluster = set->clusters + clabel;

  *weight = cluster->weight;  // 将集群的权重（样本权重之和）存储到提供的指针 weight 指向的变量中
  *mean = cluster->mean;    // 将集群的平均位置存储到提供的指针 mean 指向的变量中
  *cov = cluster->cov;      // 将集群的协方差矩阵存储到提供的指针 cov 指向的变量中
  // 如果成功获取了集群的统计信息，函数将返回 1，表示获取成功
  return 1;
}
