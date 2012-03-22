/**
 *  @file cluster.h
 *  @brief ���ͨ����֤�Ĺ켣�ľ��๦�ܡ�
 */

#ifndef _CLUSTER_H_
#define _CLUSTER_H_

#include "stdafx.h"
#include "featureTrack.h"

/**
 *  @brief  ��ɹ켣�ľ��๦��
 *
 *  @param trajectorys ��ͨ����֤�Ĺ켣��
 *  @return �������Ŀ����ʵ�ʵĳ˿���Ŀ
 */
int cluster(std::multiset<vanilla::trajectory,vanilla::_trajectoryCompare> trajectorys);

#endif
