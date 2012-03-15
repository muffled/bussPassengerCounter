/**
    featureTrack.h 用于执行特征点的检测及追踪。
    包含CTrack类。
**/

#ifndef _FEATURE_TRACK_H_
#define _FEATURE_TRACK_H_

#include <stack>
#include <vector>
#include <list>
#include <set>
#include <string>
#include "stdafx.h"

/**
    CTrack处理步骤：
    1. 对于所有轨迹的最后一个点进行追踪，更新此轨迹。如果发现此轨迹的跟踪点仍处于检测窗口(tripwire windows)中，
       并且跟踪点位置与前一帧中的位置完全一致，则删除此条轨迹；
       如果特征点跟踪失败，则删除此条轨迹。此处基于如下假设：
        * 对于图像中的噪点只有两种情况：完全静止或者跟踪失败。
        * 行人即使由于拥挤的情况而站立不动，也会存在轻微的晃动，所以特征点会存在一定的晃动，
          导致位置不会和上一帧绝对重合或者跟踪失败的情况。
    2. 为新特征点创建轨迹。如果新特征点的位置与上一帧某一特征点位置绝对重合，
       或者和某一轨迹轨迹在此帧中的追踪位置绝对重合，则不为此特征点创建轨迹。
    3. 如果某一轨迹的最后一个点已经逸出跟踪窗口，则认为此轨迹跟踪结束。
       此时对轨迹进行验证，通过验证的轨迹放入set中，用于后续的验证。
**/

namespace vanilla
{
    struct trajectory
    {
        ulong start;    // 此轨迹的开始帧
        ulong end;      // 此轨迹的结束帧
        std::vector<cv::Point2f> location;   // 此轨迹在每一帧的位置信息
    };

    class _pointCompare
    {
    public:
        bool operator () (const cv::Point2f& a,const cv::Point2f& b) const
        {
            return (a.x == b.x) ? (a.y < b.y) : (a.x < b.x);
        }
    };

    class _trajectoryCompare
    {
    public:
        bool operator () (const trajectory& a,const trajectory& b) const
        {
            return (a.start == b.start) ? (a.end < b.end) : (a.start < b.start);
        }
    };

    class CTrack
    {
    private:
        cv::Mat  m_prevFrame;   // 存储视频流的上一帧，用于后续特征点的追踪
        cv::Rect m_tripwireWindow;      // 特征点检测窗口
        cv::Rect m_trackWindow;         // 特征点追踪窗口

        // cv::goodFeaturesToTrack 特征点检测参数
        int m_maxCorners;
        double m_qualityLevel;
        double m_minDistance;

        std::list<trajectory> m_trajectorys;  // 存储视频处理中的轨迹
        std::set<cv::Point2f,_pointCompare> m_prevFrameFeatures; // 存储前一帧中的特征点
        std::multiset<trajectory,_trajectoryCompare> m_finishTra;    // 储存已经逸出跟踪窗口并且通过认证的轨迹
    protected:
        void inline SetPrevFrame(cv::Mat& curFrame) 
        {
            assert(1 == curFrame.channels() || 3 == curFrame.channels());
            if (1 == curFrame.channels()) curFrame.copyTo(m_prevFrame);
            else  cvtColor(curFrame,m_prevFrame,CV_BGR2GRAY);
        }

    public:
        CTrack(cv::Rect tripwireWindow,cv::Rect trackWindow,
               int maxCorners = 1000,double qualityLevel = 0.01,double minDistance = 3);

        void featureTrack(cv::Mat& img,ulong nFrame);  

        /**
        对于结束的轨迹，将开始帧数相同的轨迹作为一组，输出整个轨迹在不同帧数的位置信息。
        **/
        void finishedTraInfo(std::string filename = ".\\info.txt");

        /** 输出同一批次轨迹的图片。
         *      
         *  程序为每一组输出图片创建一个文件夹，用于存放该组图片
         *  @param  folder 存储输出图片的父文件夹，每一组图片的文件夹在该文件夹之下创建
         *  @param  videopath 视频文件的系统路径
         *  @param  off_frame 视频开始段舍弃的帧数
         */
        void images_byteam(std::string folder,std::string videopath,ulong off_frame);

        /** 显示出每一幅图片上增加的有效的轨迹数量，并在图片上打印出相应的数字。
         *  
         *  @param  folder 存储输出图片的父文件夹，每一组图片的文件夹在该文件夹之下创建
         *  @param  videopath 视频文件的系统路径
         *  @param  off_frame 视频开始段舍弃的帧数
         */
        void validFeatures_byFrame(std::string folder,std::string videopath,ulong off_frame);
    };
}

#endif