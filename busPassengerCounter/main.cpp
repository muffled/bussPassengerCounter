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
    //VideoCapture video("F:\\multi\\FD_001_06_00.avi");
    //VideoCapture video("F:\\multi\\FD_441_12_00.avi");
    //VideoCapture video("F:\\multi\\FD_014_10_00.avi");

    VideoCapture video("F:\\buspassenger\\2.avi");

    if (!video.isOpened())
    {
        cerr<<"can not open the video."<<endl;
        return -1;
    }

    //Rect tripwire(45,134,230,10),trackedWindow(45,106,230,28);
    Rect tripwire(45,96,230,10),trackedWindow(45,106,230,28);
    CTrack track(tripwire,trackedWindow);

    Mat _Frame,Frame;
    Mat resizeImg;
    int nFrame = 1;
    char szpath[256];
    memset(szpath,0,sizeof(szpath));
    for (int i = 0;i < 10;i++) video >> Frame;  // 去除开始的1帧
    while (true)
    {
        video >> _Frame;
        if (!_Frame.data) break;
        resize(_Frame,Frame,cv::Size(_Frame.cols >> 1,_Frame.rows >> 1));
        track.featureTrack(Frame,nFrame);

        rectangle(Frame,tripwire,CV_RGB(255,0,0));
        rectangle(Frame,trackedWindow,CV_RGB(255,255,0));

        sprintf_s(szpath,sizeof(szpath),".\\images\\2\\%d.jpg",nFrame);
        cv::resize(Frame,resizeImg,cv::Size(Frame.cols * 4,Frame.rows * 4));
        imwrite(szpath,resizeImg);

        imshow("show",Frame);
        if (waitKey(30) == 27) break;
        nFrame++;
    }
    track.finishedTraInfo();

    /*
#ifdef _TEST
    video.set(CV_CAP_PROP_POS_FRAMES,0);
    for (int i = 0;i < 25;i++) video >> _Frame;  // 去除开始的25帧
    for (int i = 1;i < 237;i++) video >> _Frame;
    size_t maxSize = 0;
    int index = 1;
    for (size_t i = 0;i < g_tra_vec.size();i++)
    {
        size_t size = g_tra_vec[i].location.size();
        if (size > maxSize) maxSize = size;
    }
    int red,green,blue;
    for (size_t i = 0;i < maxSize;i++)
    {
        video >> _Frame;
        if (!_Frame.data) break;
        resize(_Frame,Frame,cv::Size(_Frame.cols >> 1,_Frame.rows >> 1));
        for (size_t j = 0;j < g_tra_vec.size();j++)
        {
            vector<Point2f>& points = g_tra_vec[j].location;
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
        rectangle(Frame,tripwire,CV_RGB(0,0,0));
        rectangle(Frame,trackedWindow,CV_RGB(0,0,0));
        resize(Frame,_Frame,cv::Size(Frame.cols << 2,Frame.rows << 2));
        sprintf_s(szpath,sizeof(szpath),".\\temp\\%d.jpg",index++);
        imwrite(szpath,_Frame);
    }
#endif
    */

    return 1;
}
