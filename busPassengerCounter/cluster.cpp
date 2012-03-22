#include "cluster.h"


inline float _pointDistance(cv::Point2f& a,cv::Point2f& b)
{
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

/*
int cluster(std::multiset<vanilla::trajectory,vanilla::_trajectoryCompare> trajectorys)
{
    vector<clusterPoints>::size_type i = 0;

    // 取出每个轨迹的生命点(开始帧数、结束帧数）
    vector<Point> candidatePoints;
    for (list<CTrajectory>::iterator it = listTrajectory.begin();it != listTrajectory.end();it++)
    {
        Point point = it->getLife();
        candidatePoints.push_back(point);
    }
    if (0 == candidatePoints.size()) return;

    // 增加第一个聚类中心点
    clusterPoints tempCluster;
    tempCluster.m_center = candidatePoints[0];
    tempCluster.m_memberPoints.push_back(candidatePoints[0]);
    m_clusterPoints.push_back(tempCluster);

    for (vector<Point>::size_type i = 1;i < candidatePoints.size();i++)
    {
        // 提取距离候选点最近的聚类中心点
        float minDistance = 1000000000000.f;
        vector<clusterPoints>::size_type index = 0;
        for (vector<clusterPoints>::size_type j = 0;j < m_clusterPoints.size();j++)
        {
            float distance = pointDistance(candidatePoints[i],m_clusterPoints[j].m_center);
            if (distance < minDistance)
            {
                index = j;
                minDistance = distance;
            }
        }

        // 聚类
        float startDistance = (candidatePoints[i].x - m_clusterPoints[index].m_center.x) * m_timePerFrame;
        float endDistance = (candidatePoints[i].y - m_clusterPoints[index].m_center.y) * m_timePerFrame;
        if (abs(startDistance) <= 2.5 * m_varStart && abs(endDistance) <= 2.5 * m_varEnd)
        {
            // 更新聚类点中心的位置
            vector<Point>::size_type clusterSize = m_clusterPoints[index].m_memberPoints.size();
            m_clusterPoints[index].m_center.x += 1.f / (clusterSize + 1) * (candidatePoints[i].x - m_clusterPoints[index].m_center.x);
            m_clusterPoints[index].m_center.y += 1.f / (clusterSize + 1) * (candidatePoints[i].y - m_clusterPoints[index].m_center.y);
            m_clusterPoints[index].m_memberPoints.push_back(candidatePoints[i]);
        }
        else
        {
            tempCluster.m_center = candidatePoints[i];
            tempCluster.m_memberPoints.clear();   // important
            tempCluster.m_memberPoints.push_back(candidatePoints[i]);
            m_clusterPoints.push_back(tempCluster);            
        }        
    }
}
*/

int cluster(std::multiset<vanilla::trajectory,vanilla::_trajectoryCompare> trajectorys)
{
    // 取出每个轨迹的生命点(开始帧数、结束帧数）
    std::vector<cv::Point> candidatePoints;
    std::multiset<vanilla::trajectory,vanilla::_trajectoryCompare>::iterator it;
    for (it = trajectorys.begin();it != trajectorys.end();it++)
    {
        cv::Point temp(it->start,it->end);
        candidatePoints.push_back(temp);
    }
    if (0 == candidatePoints.size()) return 0;

    return 1;
}