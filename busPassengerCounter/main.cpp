#include "stdafx.h"
#include "featureTrack.h"
#include <iostream>

using namespace std;
using namespace cv;
using namespace vanilla;

#define _TEST

#ifdef _TEST
extern std::vector<vanilla::trajectory> g_tra_vec;
#endif


int main()
{
    VideoCapture video("F:\\multi\\FD_001_06_00.avi");
    //VideoCapture video("F:\\multi\\FD_441_12_00.avi");
    //VideoCapture video("F:\\multi\\FD_014_10_00.avi");

    //VideoCapture video("F:\\buspassenger\\2.avi");

    if (!video.isOpened())
    {
        cerr<<"can not open the video."<<endl;
        return -1;
    }

    Rect tripwire(45,134,230,10),trackedWindow(45,106,230,28);
    //Rect tripwire(45,96,230,10),trackedWindow(45,106,230,28);
    CTrack track(tripwire,trackedWindow);

    Mat _Frame,Frame;
    Mat resizeImg;
    int nFrame = 0;
    char szpath[256];
    memset(szpath,0,sizeof(szpath));
    for (int i = 0;i < 10;i++) video >> Frame;  // 去除开始的10帧
    while (true)
    {
//         video >> _Frame;
//         if (!_Frame.data) break;
//         resize(_Frame,Frame,cv::Size(_Frame.cols >> 1,_Frame.rows >> 1));
        video >> Frame;
        if (!Frame.data) break;
        track.featureTrack(Frame,nFrame);

        rectangle(Frame,tripwire,CV_RGB(255,0,0));
        rectangle(Frame,trackedWindow,CV_RGB(255,255,0));

//         sprintf_s(szpath,sizeof(szpath),".\\images\\2\\%d.jpg",nFrame);
//         cv::resize(Frame,resizeImg,cv::Size(Frame.cols * 4,Frame.rows * 4));
//         imwrite(szpath,resizeImg);

        imshow("show",Frame);
        if (waitKey(30) == 27) break;
        nFrame++;
    }

//     track.finishedTraInfo();
// 
//     string folder("F:\\buspassenger\\speed\\FD_441_12_00_2");
//     string videopath("F:\\multi\\FD_441_12_00.avi");
//     int off_frame = 10;
//     track.images_byteam(folder,videopath,off_frame);

    string folder("F:\\buspassenger\\validFeatures\\FD_001_06_00_8");
    string videopath("F:\\multi\\FD_001_06_00.avi");
    int off_frame = 10;
    track.validFeatures_byFrame(folder,videopath,off_frame);
    
    return 1;
}
