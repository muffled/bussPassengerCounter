#include "stdafx.h"
#include "featureTrack.h"
#include <iostream>

using namespace std;
using namespace cv;
using namespace vanilla;

int main()
{
    VideoCapture video("F:\\multi\\FD_001_06_00.avi");
    if (!video.isOpened())
    {
        cerr<<"can not open the video."<<endl;
        return -1;
    }

    Rect tripwindow(45,134,230,10),trackwindow(45,106,230,28);
    CTrack track(tripwindow,trackwindow);

    Mat Frame; 
    ulong nFrame = 0;
    for (int i = 0;i < 10;i++) video >> Frame;    // 去除开始的10帧
    while (true)
    {
        video >> Frame;
        if (!Frame.data) break;
        track.featureTrack(Frame,nFrame++,PASSENGER_UP);

        rectangle(Frame,tripwindow,CV_RGB(255,255,0));
        rectangle(Frame,trackwindow,CV_RGB(0,0,255));

        imshow("show",Frame);
        if (waitKey(10) == 27) break;      
    }

    track.verify(251,140);

    return 1;
}
