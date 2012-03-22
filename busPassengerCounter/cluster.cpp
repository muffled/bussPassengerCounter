#include "cluster.h"


inline float _pointDistance(cv::Point2f& a,cv::Point2f& b)
{
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

/*
int cluster(std::multiset<vanilla::trajectory,vanilla::_trajectoryCompare> trajectorys)
{
    vector<clusterPoints>::size_type i = 0;

    // ȡ��ÿ���켣��������(��ʼ֡��������֡����
    vector<Point> candidatePoints;
    for (list<CTrajectory>::iterator it = listTrajectory.begin();it != listTrajectory.end();it++)
    {
        Point point = it->getLife();
        candidatePoints.push_back(point);
    }
    if (0 == candidatePoints.size()) return;

    // ���ӵ�һ���������ĵ�
    clusterPoints tempCluster;
    tempCluster.m_center = candidatePoints[0];
    tempCluster.m_memberPoints.push_back(candidatePoints[0]);
    m_clusterPoints.push_back(tempCluster);

    for (vector<Point>::size_type i = 1;i < candidatePoints.size();i++)
    {
        // ��ȡ�����ѡ������ľ������ĵ�
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

        // ����
        float startDistance = (candidatePoints[i].x - m_clusterPoints[index].m_center.x) * m_timePerFrame;
        float endDistance = (candidatePoints[i].y - m_clusterPoints[index].m_center.y) * m_timePerFrame;
        if (abs(startDistance) <= 2.5 * m_varStart && abs(endDistance) <= 2.5 * m_varEnd)
        {
            // ���¾�������ĵ�λ��
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
    // ȡ��ÿ���켣��������(��ʼ֡��������֡����
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