#include "featureTrack.h"
#include "vani_log.h"
#include <iostream>
#include <iomanip>

#define _TEST
std::vector<vanilla::trajectory> g_tra_vec;
#ifdef _TEST

#endif

vanilla::CTrack::CTrack(cv::Rect tripwireWindow,cv::Rect trackWindow, 
                        int maxCorners/* = 1000*/,double qualityLevel/* = 0.01*/,double minDistance/* = 3*/)
{
    m_tripwireWindow = tripwireWindow;
    m_trackWindow = trackWindow;

    m_maxCorners = maxCorners;
    m_qualityLevel = qualityLevel;
    m_minDistance = minDistance;
}

std::ofstream& operator << (std::ofstream& os,cv::Point2f& point)
{
    //os<<"("<<point.x<<","<<point.y<<")";
    os<<"(";
    os<<setiosflags(std::ios::right)<<setiosflags(std::ios::fixed);
    os<<std::setprecision(3);
    os<<std::setw(7)<<point.x;
    os<<",";
    os<<std::setw(7)<<point.y;
    os<<")";
    return os;
}

void vanilla::CTrack::featureTrack(cv::Mat& img,ulong nFrame)
{
    cv::Mat curGrayFrame;
    assert(1 == img.channels() || 3 == img.channels());
    if (1 == img.channels())  img.copyTo(curGrayFrame);
    else
        cvtColor(img,curGrayFrame,CV_BGR2GRAY);

    // 特征点的追踪
    std::set<cv::Point2f,_pointCompare> curTrackedFeatures;  // 存储当前追踪到的仍处于tripwire中的特征点位置
    std::vector<cv::Point2f> tobeTrackedFeatures;   // 存储当前轨迹链表中每一轨迹的最后一个点，用于追踪
    std::list<vanilla::trajectory>::iterator tra_it;

#ifdef _TEST
    // 记录下每帧追踪点的数目
    static std::ofstream trackQualityFile(".\\temp\\trackQuality.txt");
#endif

#ifdef _TEST
    // 记录下每帧中每个轨迹开始的帧时间
    static std::ofstream startFrameFile(".\\temp\\startFrame.txt");
#endif   

#ifdef _TEST

#endif

    if (m_prevFrame.data)
    {
        for (tra_it = m_trajectorys.begin();tra_it != m_trajectorys.end();tra_it++)
            tobeTrackedFeatures.push_back(tra_it->location[tra_it->location.size() - 1]);
/*
#ifdef _TEST
        std::vector<ulong> startFrame;
        for (tra_it = m_trajectorys.begin();tra_it != m_trajectorys.end();tra_it++)
            startFrame.push_back(tra_it->start);
        std::sort(startFrame.begin(),startFrame.end());
        for (size_t i = 0;i < startFrame.size();i++)
            startFrameFile<<startFrame[i]<<" ";
        startFrameFile<<std::endl;

        ulong curFrame = 0;//startFrame[startFrame.size() - 1];
        if (startFrame.size() > 0)
            curFrame = startFrame[startFrame.size() - 1];
        for (tra_it = m_trajectorys.begin();tra_it != m_trajectorys.end();)
        {
            ulong start = tra_it->start;
            if (curFrame - start > 50) tra_it = m_trajectorys.erase(tra_it);
            else
                tra_it++;
        }

        for (tra_it = m_trajectorys.begin();tra_it != m_trajectorys.end();tra_it++)
            tobeTrackedFeatures.push_back(tra_it->location.top());
#endif
*/


#ifdef _TEST
        trackQualityFile<<tobeTrackedFeatures.size()<<" ";
#endif

        std::vector<cv::Point2f> trackedLocation;  // 追踪到的位置
        std::vector<uchar> status;
        std::vector<float> err;        
        cv::calcOpticalFlowPyrLK(m_prevFrame,curGrayFrame,tobeTrackedFeatures,trackedLocation,status,err); 
        
        // 如果追踪失败或者追踪点与前一帧完全一致，则删除掉此条轨迹
        tra_it = m_trajectorys.begin();
        for (size_t i = 0;i < status.size();i++)
        {
            if (!status[i]) tra_it = m_trajectorys.erase(tra_it);  // 追踪失败
            else
            {  
                // 特征点仍处于检测窗口中，且追踪位置与前一帧完全重合
                if ( m_tripwireWindow.contains(trackedLocation[i])
                    && cvRound(tobeTrackedFeatures[i].x) == cvRound(trackedLocation[i].x)
                    && cvRound(tobeTrackedFeatures[i].y) == cvRound(trackedLocation[i].y) )                 
                {
                    tra_it = m_trajectorys.erase(tra_it);
                    continue;
                }
                tra_it->location.push_back(trackedLocation[i]);  // 更新轨迹

                if (m_tripwireWindow.contains(trackedLocation[i])) 
                {
                    curTrackedFeatures.insert(trackedLocation[i]);
                    tra_it++;
                    continue;
                }

                if (!m_trackWindow.contains(trackedLocation[i]))  // 已经逸出追踪窗口                   
                {
//                     if (trackedLocation[i].y < m_trackWindow.y)
//                     {
//                         // 保存下已经完成的轨迹，用于后续的聚类。
//                         tra_it->end = nFrame;   // 存储下结束的帧数
//                         m_finishTra.insert(*tra_it);
//                     }
                    if (trackedLocation[i].y > m_trackWindow.y)
                    {
                        // 保存下已经完成的轨迹，用于后续的聚类。
                        tra_it->end = nFrame;   // 存储下结束的帧数
                        m_finishTra.insert(*tra_it);
                    }
                    tra_it = m_trajectorys.erase(tra_it);
                    continue;
                }

                // 特征点仍处于追踪窗口内
                tra_it++;
            }               
        }
    }


    // 特征点的检测
    std::vector<cv::Point2f> tripwireCorners,detectedFeatures;
    cv::goodFeaturesToTrack(curGrayFrame(m_tripwireWindow),tripwireCorners,m_maxCorners,
                            m_qualityLevel,m_minDistance,cv::Mat(),3);

    // offPoint:检测时所传入的图片，为原图中tripwire window所在的部分。
    //          利用cv::goodFeaturesToTrack 所检测到的特征点坐标是相对于tripwire window而言。
    //          故在存储坐标时，需要加上offPoint，将检测到的坐标还原到原图中。
    cv::Point offPoint(m_tripwireWindow.x,m_tripwireWindow.y);
    for (std::vector<cv::Point2f>::size_type i = 0;i < tripwireCorners.size();i++)
    {
        cv::Point2f point(tripwireCorners[i].x + offPoint.x,tripwireCorners[i].y + offPoint.y);
        detectedFeatures.push_back(point);
    }

    static std::set<cv::Point2f,_pointCompare> prevDetectedFeatures,curDetectedFeatures;
    static std::set<cv::Point2f,_pointCompare>::iterator set_it;
    curDetectedFeatures.clear();
    for (size_t i = 0;i < detectedFeatures.size();i++)
        curDetectedFeatures.insert(detectedFeatures[i]);

#ifdef _TEST
    static std::ofstream addNumberFile("addNumber.txt");
    int naddNumber = 0;
#endif

    for (size_t i = 0;i < detectedFeatures.size();i++)
    {
        set_it = prevDetectedFeatures.find(detectedFeatures[i]);
        if (set_it == prevDetectedFeatures.end()) 
        {
            set_it = curTrackedFeatures.find(detectedFeatures[i]);
            if (set_it == curTrackedFeatures.end())
            {
                trajectory tmp;
                tmp.start = nFrame;
                tmp.location.push_back(detectedFeatures[i]);
                m_trajectorys.push_back(tmp);

#ifdef _TEST
                naddNumber++;
                cv::circle(img,detectedFeatures[i],1,CV_RGB(255,255,0),-1);
#endif
                
            } 

#ifdef _TEST
            else
                cv::circle(img,detectedFeatures[i],1,CV_RGB(0,0,255),-1);
#endif
        }

#ifdef _TEST
        else
            cv::circle(img,detectedFeatures[i],1,CV_RGB(0,0,255),-1);
#endif

    }

#ifdef _TEST
    addNumberFile<<"add number:"<<naddNumber<<std::endl;
#endif

    prevDetectedFeatures.swap(curDetectedFeatures);
    SetPrevFrame(curGrayFrame);
}

void _outputTraLocation(std::ofstream& os,std::vector<vanilla::trajectory>& tra_vec)
{
    size_t maxLength = 0;
    for (size_t i = 0;i < tra_vec.size();i++)
    {
        size_t size = tra_vec[i].location.size();
        if (size > maxLength) maxLength = size;
    }

    for (size_t i = 0;i < maxLength;i++)
    {
        for (size_t j = 0;j < tra_vec.size();j++)
        {
            std::vector<cv::Point2f>& points = tra_vec[j].location;
            if (points.size() > i)
                os<<points[i]<<" ";
            else
                os<<"         "<<" "<<"        ";
        }
        os<<std::endl;
    }
    os<<std::endl;
}

void vanilla::CTrack::finishedTraInfo(std::string filename /* = */ )
{
    std::ofstream outfile(filename.c_str());
    std::vector<vanilla::trajectory> tra_vec;
    std::multiset<trajectory,_trajectoryCompare>::iterator it;

    it = m_finishTra.begin();
    ulong TeamFrame = it->start;
    int nteam = 1;
    for (;it != m_finishTra.end();it++)
    {
        if (TeamFrame == it->start) tra_vec.push_back(*it);
        else
        {
            outfile<<"Team:"<<nteam++<<",start Frame:"<<TeamFrame<<std::endl;
            _outputTraLocation(outfile,tra_vec);

#ifdef _TEST
            //if (TeamFrame == 237) g_tra_vec = tra_vec;
#endif
            tra_vec.clear();    
            tra_vec.push_back(*it);
            TeamFrame = it->start;
        }
    }
}