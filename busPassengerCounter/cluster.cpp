#include "cluster.h"

struct colorPoint
{
	int m_red;
	int m_green;
	int m_blue;
	cv::Point2f m_point;
}; 

inline float _pointDistance(cv::Point2f& a,cv::Point2f& b)
{
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

void validFeatures(std::string videopath,ulong offFrame,std::string folder,
				   cv::Rect& tripwindow,cv::Rect& trackwindow,
				   std::vector<std::vector<vanilla::trajectory> > clusters)
{
	std::map<ulong,std::vector<colorPoint> > validFeatures_perFrame;
	std::map<ulong,std::vector<colorPoint> >::iterator it;	
	
	for (std::size_t i = 0;i < clusters.size();i++)
	{
		std::vector<vanilla::trajectory>& cluster = clusters[i];
		int red = 0,green = 0,blue = 0;
		// 每一个聚类按照红、绿、蓝依次赋色
		if (i % 3 == 0) red = 255;
		else if (i % 3 == 1) green = 255;
		else blue = 255;

		for (std::size_t j = 0;j < cluster.size();j++)
		{
			vanilla::trajectory& tra = cluster[j];
			ulong start = tra.start;
			std::vector<cv::Point2f> points = tra.location;

			for (std::size_t k = 0;k < points.size();k++)
			{
				colorPoint tmp;
				tmp.m_red = red;
				tmp.m_green = green;
				tmp.m_blue = blue;
				tmp.m_point = points[k];

				it = validFeatures_perFrame.find(k + start);
				if (it != validFeatures_perFrame.end())
					it->second.push_back(tmp);
				else
				{
					std::vector<colorPoint> tmp_vec;
					tmp_vec.push_back(tmp);
					validFeatures_perFrame.insert(std::pair<ulong,std::vector<colorPoint> >(k + start,tmp_vec));
				}
			}
		}
	}

	// 输出每一帧图片
	cv::VideoCapture video(videopath);
	if (!video.isOpened())
	{
		std::cerr<<"can not open the vidoe."<<std::endl;
		return;
	}
	cv::Mat Frame,_Frame;	
	char szpath[256];
	memset(szpath,0,sizeof(szpath));
	ulong nFrame = 0;
	for (ulong i = 0;i < offFrame;i++) video >> Frame;
	while (true)
	{
		video >> Frame;
		if (!Frame.data) break;
		it = validFeatures_perFrame.find(nFrame);
		if (it != validFeatures_perFrame.end())
		{
			std::vector<colorPoint>& colorPoints = it->second;
			for (std::size_t i = 0;i < colorPoints.size();i++)
			{
				int red = colorPoints[i].m_red;
				int green = colorPoints[i].m_green;
				int blue = colorPoints[i].m_blue;
				cv::Point2f point = colorPoints[i].m_point;
				circle(Frame,point,1,CV_RGB(red,green,blue),-1);
			}
		}
		rectangle(Frame,tripwindow,CV_RGB(0,0,0));
		rectangle(Frame,trackwindow,CV_RGB(0,0,0));
		cv::resize(Frame,_Frame,cv::Size(Frame.cols << 1,Frame.rows << 1));
		sprintf_s(szpath,"%s\\%d.jpg",folder.c_str(),nFrame);
		cv::imwrite(szpath,_Frame);
		cv::imshow("show",Frame);
		if (cv::waitKey(10) == 27) break;
		nFrame++;
	}
}

std::vector<std::vector<vanilla::trajectory> > cluster(std::multiset<vanilla::trajectory,vanilla::_trajectoryCompare> trajectorys,
													   ulong start_dev,ulong end_dev,int time_perFrame,int threshold)
{
	/**
	 *	STL multiset 中的元素按照一定的顺序存储。
	 *  按照轨迹生命开始期从小到大存储，如果两个轨迹生命开始期相同，则按照轨迹生命结束期从小到大存储。
	 *  聚类时，可以利用此一性质。
	 */
	std::vector<std::vector<vanilla::trajectory> > clusters;
	std::vector<vanilla::trajectory> cluster_temp;
    std::multiset<vanilla::trajectory,vanilla::_trajectoryCompare>::iterator it;

	if (trajectorys.size() == 0) return clusters;

	it = trajectorys.begin();
	cv::Point2f center((float)it->start,(float)it->end);
	cluster_temp.push_back(*it);
	it++;
    for (;it != trajectorys.end();it++)
    {
		float d_start = abs( (it->start - center.x) * time_perFrame );
		float d_end = abs( (it->end - center.y) * time_perFrame );
		if (d_start < 2.5f*start_dev && d_end < 2.5f*end_dev)
		{
			float size = (float)(cluster_temp.size());
			center.x += 1.f * (it->start - center.x) / (size + 1);
			center.y += 1.f * (it->end - center.y) / (size + 1);
			cluster_temp.push_back(*it);
		}
		else	// 后续的点已经都不属于先前的聚类
		{
			if ( (int)cluster_temp.size() >= threshold ) clusters.push_back(cluster_temp);
			// 新的聚类
			cluster_temp.clear();
			center.x = (float)it->start;
			center.y = (float)it->end;
			cluster_temp.push_back(*it);
		}
    }

	// 每一个聚类结果存储至一个文本文件中
	for (size_t i = 0;i < clusters.size();i++)
	{
		std::vector<vanilla::trajectory>& cluster = clusters[i];

		char szfile[256];
		memset(szfile,0,sizeof(szfile));
		sprintf_s(szfile,sizeof(szfile),".\\temp\\%d.txt",i);

		std::ofstream file(szfile);
		for (size_t j = 0;j < cluster.size();j++)
		{
			file<<cluster[j].start<<std::endl;
			file<<cluster[j].end<<std::endl;
		}
	}
	
    return clusters;
}