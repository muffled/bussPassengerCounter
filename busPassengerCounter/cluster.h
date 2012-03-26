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
 *  @param trajectorys  ��ͨ����֤�Ĺ켣��
 *	@param start_dev	�켣������ʼ�ھ��෽��
 *	@param end_dev		�켣���������ھ��෽��
 *  @param time_perFrame������֮֡���ʱ����
 *	@param threshold	ÿ����Ч������Ҫ��������С��Ԫ����Ŀ
 *
 *  @return ����Ľ��
 */
std::vector<std::vector<vanilla::trajectory> > cluster(std::multiset<vanilla::trajectory,vanilla::_trajectoryCompare> trajectorys,
													   ulong start_dev,ulong end_dev,int time_perFrame,int threshold);


/**
 *	@brief	���ݾ�����������Ƶÿһ֡����������е�������
 *
 *	@param videopath	��Ƶ��ϵͳ·��
 *	@param offFrame		��Ƶ��ʼһ��������֡��
 *	@param folder		ͼ��Ĵ洢�ļ���
 *	@param tripWindow	�������ⴰ��
 *	@param trackWindow	��������ٴ���
 *	@param cluster		����Ľ��
 */
void validFeatures(std::string videopath,ulong offFrame,std::string folder,
				   cv::Rect& tripWindow,cv::Rect& trackWindow,
				   std::vector<std::vector<vanilla::trajectory> > clusters);

#endif
