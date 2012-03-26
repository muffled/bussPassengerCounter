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
 *  @param trajectorys  已通过验证的轨迹。
 *	@param start_dev	轨迹生命开始期聚类方差
 *	@param end_dev		轨迹生命结束期聚类方差
 *  @param time_perFrame相邻两帧之间的时间间隔
 *	@param threshold	每个有效聚类需要包括的最小的元素数目
 *
 *  @return 聚类的结果
 */
std::vector<std::vector<vanilla::trajectory> > cluster(std::multiset<vanilla::trajectory,vanilla::_trajectoryCompare> trajectorys,
													   ulong start_dev,ulong end_dev,int time_perFrame,int threshold);


/**
 *	@brief	根据聚类结果，在视频每一帧上输出聚类中的特征点
 *
 *	@param videopath	视频的系统路径
 *	@param offFrame		视频开始一段舍弃的帧数
 *	@param folder		图像的存储文件夹
 *	@param tripWindow	特征点检测窗口
 *	@param trackWindow	特征点跟踪窗口
 *	@param cluster		聚类的结果
 */
void validFeatures(std::string videopath,ulong offFrame,std::string folder,
				   cv::Rect& tripWindow,cv::Rect& trackWindow,
				   std::vector<std::vector<vanilla::trajectory> > clusters);

#endif
