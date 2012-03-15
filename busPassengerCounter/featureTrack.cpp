#include "featureTrack.h"
#include "vani_log.h"
#include <direct.h>
#include <iostream>
#include <iomanip>


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


    if (m_prevFrame.data)
    {
        for (tra_it = m_trajectorys.begin();tra_it != m_trajectorys.end();tra_it++)
            tobeTrackedFeatures.push_back(tra_it->location[tra_it->location.size() - 1]);

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

                if ( tobeTrackedFeatures[i].y - trackedLocation[i].y > m_trackWindow.height / 2)
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

                // 在跟踪窗口中，如果发现特征点向后运动，则简单的抛弃此条轨迹
                // 此处先测试此条粗暴假设的结果
                if ( m_trackWindow.contains(trackedLocation[i]) &&
                    (tobeTrackedFeatures[i].y < trackedLocation[i].y) )
                {
                    tra_it = m_trajectorys.erase(tra_it);
                    continue;
                }

                if (!m_trackWindow.contains(trackedLocation[i]))  // 已经逸出追踪窗口                   
                {
                    if (trackedLocation[i].y < m_trackWindow.y)
                    {
                        // 保存下已经完成的轨迹，用于后续的聚类。
                        tra_it->end = nFrame;   // 存储下结束的帧数
                        m_finishTra.insert(*tra_it);
                    }
//                     if (trackedLocation[i].y > m_trackWindow.y)
//                     {
//                         // 保存下已经完成的轨迹，用于后续的聚类。
//                         tra_it->end = nFrame;   // 存储下结束的帧数
//                         m_finishTra.insert(*tra_it);
//                     }

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
            } 
        }
    }

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
            tra_vec.clear();    
            tra_vec.push_back(*it);
            TeamFrame = it->start;
        }
    }
}


void _outputTeamImage(cv::VideoCapture& video,std::vector<vanilla::trajectory>& tra_vec,
                      int start_fream,int off_frame,std::string folder,
                      cv::Rect& trip_rect,cv::Rect& track_rect)
{
    static int team = 1;

    video.set(CV_CAP_PROP_POS_FRAMES,off_frame + start_fream);   // 调整到轨迹开始的位置

    size_t maxLength = 0;
    for (size_t i = 0;i < tra_vec.size();i++)
    {
        size_t size = tra_vec[i].location.size();
        if (size > maxLength) maxLength = size;
    }    

    int red,green,blue;
    int index = 1;
    char szpath[256];
    memset(szpath,0,sizeof(szpath));
    cv::Mat Frame,_Frame;
    for (std::size_t i = 0;i < maxLength;i++)
    {
        video >> Frame;
        if (!Frame.data) break;
        //resize(_Frame,Frame,cv::Size(_Frame.cols >> 1,_Frame.rows >> 1));
        for (std::size_t j = 0;j < tra_vec.size();j++)
        {
            std::vector<cv::Point2f>& points = tra_vec[j].location;
            if (j % 4 == 0)
            {
                red = 255;
                blue = green = 0;
            }
            else if (j % 4 == 1)
            {
                red = 0;
                green = 255;
                blue = 0;
            }
            else if (j % 4 == 2)
            {
                red = green = 0;
                blue = 255;
            }
            else
            {
                red = green = 255;
                blue = 0;
            }
            if (points.size() > i)
                circle(Frame,points[i],1,CV_RGB(red,green,blue),-1);
            else
                circle(Frame,points[points.size() - 1],1,CV_RGB(red,green,blue),-1);
        }
        cv::rectangle(Frame,trip_rect,CV_RGB(0,0,0));
        cv::rectangle(Frame,track_rect,CV_RGB(0,0,0));
        cv::resize(Frame,_Frame,cv::Size(Frame.cols << 2,Frame.rows << 2));
        //sprintf_s(szpath,sizeof(szpath),"%s\\%d.jpg",folder.c_str(),index++);
        sprintf_s(szpath,sizeof(szpath),"%s\\%d_%d.jpg",folder.c_str(),team,index++);
        cv::imwrite(szpath,_Frame);
    }
    team++;
}

void vanilla::CTrack::images_byteam(std::string folder,std::string videopath,ulong off_frame)
{   
    // 将轨迹按组存储
    std::vector<std::vector<vanilla::trajectory> > tra_teams;   
    std::multiset<trajectory,_trajectoryCompare>::iterator it;

    std::vector<vanilla::trajectory> temp;
    it = m_finishTra.begin();
    ulong TeamFrame = it->start;
    for (;it != m_finishTra.end();it++)
    {
        if (TeamFrame == it->start) temp.push_back(*it);
        else
        {
            tra_teams.push_back(temp);
            temp.clear();    
            temp.push_back(*it);
            TeamFrame = it->start;
        }
    }

    // 输出每组轨迹
    cv::VideoCapture video(videopath);
    int index = 1;
    char szdir[256];
    memset(szdir,0,sizeof(szdir));
    for (size_t i = 0;i < tra_teams.size();i++)
    {
        std::cout<<"left:"<<tra_teams.size() - i<<std::endl;
        if (tra_teams[i].size() < 8) continue;
        sprintf_s(szdir,sizeof(szdir),"%s\\%d",folder.c_str(),index++);
        _mkdir(szdir);
//         _outputTeamImage(video,tra_teams[i],tra_teams[i][0].start,off_frame,std::string(szdir),
//                          m_tripwireWindow,m_trackWindow);
        _outputTeamImage(video,tra_teams[i],tra_teams[i][0].start,off_frame,folder,
            m_tripwireWindow,m_trackWindow);
    }
}

void vanilla::CTrack::validFeatures_byFrame(std::string folder,std::string videopath,ulong off_frame)
{
    if (m_finishTra.size() == 0) return;   

    cv::VideoCapture video(videopath);
    cv::Mat Frame,_Frame;
    int index = 1;
    char szpath[256];
    memset(szpath,0,sizeof(szpath));
    char szinfo[256];
    memset(szinfo,0,sizeof(szinfo));

    std::multiset<trajectory,_trajectoryCompare>::iterator it;
    std::vector<cv::Point2f> points;
    it = m_finishTra.begin();
    ulong start_frame = it->start;
    int count = 0;
    int size = m_finishTra.size();
    
    for (;it != m_finishTra.end();it++)
    {
        std::cout<<"left:"<<size - count++<<std::endl;
        if (start_frame == it->start) points.push_back(it->location[0]);
        else
        {
            if (points.size() >= 8)
            {
                video.set(CV_CAP_PROP_POS_FRAMES,off_frame + start_frame);   // 调整到轨迹开始的位置
                video >> Frame;
                for (std::size_t i = 0;i < points.size();i++)
                    cv::circle(Frame,points[i],1,CV_RGB(0,0,255),-1);
                cv::rectangle(Frame,m_tripwireWindow,CV_RGB(0,0,0));
                cv::rectangle(Frame,m_trackWindow,CV_RGB(0,0,0));

                sprintf_s(szinfo,sizeof(szinfo),"Frame:%d",start_frame);
                cv::putText(Frame,szinfo,cv::Point(10,30),cv::FONT_HERSHEY_PLAIN,1,CV_RGB(255,255,0));
                sprintf_s(szinfo,sizeof(szinfo),"features quantity:%d",points.size());
                cv::putText(Frame,szinfo,cv::Point(10,50),cv::FONT_HERSHEY_PLAIN,1,CV_RGB(255,255,0));

                cv::resize(Frame,_Frame,cv::Size(Frame.cols << 1,Frame.rows << 1));
                sprintf_s(szpath,sizeof(szpath),"%s\\%d.jpg",folder.c_str(),index++);
                cv::imwrite(szpath,_Frame);
            }

            points.clear();
            points.push_back(it->location[0]);
            start_frame = it->start;          
        }
    }
}