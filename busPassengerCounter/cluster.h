/**
 *  @file cluster.h
 *  @brief 完成通过验证的轨迹的聚类功能。
 */

#ifndef _CLUSTER_H_
#define _CLUSTER_H_

#include "stdafx.h"
#include "featureTrack.h"

/**
 *  @brief  完成轨迹的聚类功能
 *
 *  @param trajectorys 已通过验证的轨迹。
 *  @return 聚类的数目，即实际的乘客数目
 */
int cluster(std::multiset<vanilla::trajectory,vanilla::_trajectoryCompare> trajectorys);

#endif
