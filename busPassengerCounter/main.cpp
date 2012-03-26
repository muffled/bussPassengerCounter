#include "stdafx.h"
#include "featureTrack.h"
#include "cluster.h"
#include <iostream>

using namespace std;
using namespace cv;
using namespace vanilla;


int main()
{
    string videopath("G:\\work\\buspassenger\\testvideos\\front\\FD_001_06_00.avi");
    VideoCapture video(videopath);
    if (!video.isOpened())
    {
        cerr<<"can not open the video."<<endl;
        return -1;
    }

    Rect tripwindow(45,134,230,10),trackwindow(45,106,230,28);
    CTrack track(tripwindow,trackwindow);

    Mat Frame; 
    ulong nFrame = 0;
	//char szpath[256];
	//memset(szpath,0,sizeof(szpath));
    for (int i = 0;i < 10;i++) video >> Frame;    // 去除开始的10帧
    while (true)
    {
        video >> Frame;
        if (!Frame.data) break;
        //track.featureTrack(Frame,nFrame++,PASSENGER_UP,int(1));
		track.featureTrack(Frame,nFrame++,PASSENGER_UP);

        rectangle(Frame,tripwindow,CV_RGB(255,255,0));
        rectangle(Frame,trackwindow,CV_RGB(0,0,255));

        imshow("show",Frame);
		//sprintf_s(szpath,sizeof(szpath),"G:\\work\\buspassenger\\images\\FD_001_06_00\\%d.jpg",nFrame);
		//imwrite(szpath,Frame);
        if (waitKey(10) == 27) break;       
    }

 	double mean = 251,std_dev = 150;
    //track.verify(mean,std_dev,int(1));
	track.verify(mean,std_dev);

	ulong start_dev = 364,end_dev = 359;
	int time_perFrame = 40;
	int threshold = 20;
	multiset<vanilla::trajectory,_trajectoryCompare> tras = track.trajectorys();
	vector<vector<vanilla::trajectory> > clusters;
	clusters = cluster(tras,start_dev,end_dev,time_perFrame,threshold);

	string folder("G:\\work\\buspassenger\\clusterResult\\FD_001_06_00_old");
	ulong offFrame = 10;
	validFeatures(videopath,offFrame,folder,tripwindow,trackwindow,clusters);

    return 1;
}


/*
int main()
{
	string videopath("G:\\work\\buspassenger\\testvideos\\front\\FD_001_06_00.avi");
	VideoCapture video(videopath);
	if (!video.isOpened())
	{
		cerr<<"can not open the video."<<endl;
		return -1;
	}

	Mat Frame;
	char szpath[256];
	memset(szpath,0,sizeof(szpath));
	int index = 1;
	while (true)
	{
		video >> Frame;
		sprintf_s(szpath,sizeof(szpath),"g:\\test\\%d.jpg",index++);
		imwrite(szpath,Frame);
		imshow("show",Frame);
		if (waitKey(10) > 0) break;
	}

	return 1;
}
*/
