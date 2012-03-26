/** 
 *  @file featureTrack.h
 *  @brief   ʵ�ֿ���ͳ���㷨�е��������⼰׷�ٵ�ģ��.
 *
 *  ���ģ�Clustering method for counting passengers getting in a bus with single camera
 *  ��Ҫͨ���� CTrack ʵ��.
 */

#ifndef _FEATURE_TRACK_H_
#define _FEATURE_TRACK_H_

#include "stdafx.h"

namespace vanilla
{

#define PASSENGER_UP   1
#define PASSENGER_DOWN 2

    struct trajectory
    {
        ulong start;    /**< �˹켣�Ŀ�ʼ֡ */
        ulong end;      /**< �˹켣�Ľ���֡ */
        std::vector<cv::Point2f> location;   /**< �˹켣��ÿһ֡��λ����Ϣ */
    };

	struct features
	{
		ulong frame;
		std::vector<cv::Point2f> tripwire_features;
		std::vector<cv::Point2f> track_features;
	};



    /** 
     *  @brief ���� std::set<cv::Point2f> �ıȽ��� 
     */
    class _pointCompare
    {
    public:
        bool operator () (const cv::Point2f& a,const cv::Point2f& b) const
        {
            return (a.x == b.x) ? (a.y < b.y) : (a.x < b.x);
        }
    };

    /**
     *  @brief ���� std::multiset<vanilla::trajectory> �ıȽ���
     */
    class _trajectoryCompare
    {
    public:
        bool operator () (const trajectory& a,const trajectory& b) const
        {
            return (a.start == b.start) ? (a.end < b.end) : (a.start < b.start);
        }
    };

/**
 *   CTrack�����裺
 *   1. �������й켣�����һ�������׷�٣����´˹켣��������ִ˹켣�ĸ��ٵ��Դ��ڼ�ⴰ��(tripwire windows)�У�
 *      ���Ҹ��ٵ�λ����ǰһ֡�е�λ����ȫһ�£���ɾ�������켣��
 *      ������������ʧ�ܣ���ɾ�������켣���˴��������¼��裺
 *       * ����ͼ���е����ֻ�������������ȫ��ֹ���߸���ʧ�ܡ�
 *       * ���˼�ʹ����ӵ���������վ��������Ҳ�������΢�Ļζ�����������������һ���Ļζ���
 *         ����λ�ò������һ֡�����غϻ��߳��ָ���ʧ�ܵ������
 *   2. Ϊÿһ֡�м�⵽���������㴴���켣��
 *      ������������λ������һ֡ĳһ������λ�þ����غϣ�
 *      ���ߺ�ĳһ�켣�켣�ڴ�֡�е�׷��λ�þ����غϣ���Ϊ�������㴴���켣��
 *   3. ���ĳһ�켣�����һ�����Ѿ��ݳ����ٴ��ڣ�����Ϊ�˹켣���ٽ�����
 *      ��ʱ�Թ켣������֤��ͨ����֤�Ĺ켣����set�У����ں�������֤��
 */
    class CTrack
    {
    private:
        cv::Mat  m_prevFrame;   /**> �洢��Ƶ������һ֡�����ں����������׷�� */
        cv::Rect m_tripwireWindow;      /**> �������ⴰ�� */
        cv::Rect m_trackWindow;         /**> ������׷�ٴ��� */

        /** cv::goodFeaturesToTrack ����������� */
        int m_maxCorners;
        double m_qualityLevel;
        double m_minDistance;

        std::list<trajectory> m_trajectorys;  /**> �洢��Ƶ�����еĹ켣 */
        std::set<cv::Point2f,_pointCompare> m_prevFrameFeatures; /**> �洢ǰһ֡�е������� */
        /** �����Ѿ��ݳ����ٴ��ڲ���ͨ����֤�Ĺ켣 */
        std::multiset<trajectory,_trajectoryCompare> m_finishTra;    
        /** ���Ѿ���ɵ�δͨ����֤�Ĺ켣��ӵ������� */
        std::list<trajectory> m_finishTra_list;  
		/** ��¼��ÿһ֡����Ч�������� */
		std::vector<features> m_features_vec;
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

        /** 
         *  @brief ������ļ�⼰׷��
         *
         *  ��⼰���ٵĹ�����������ע��˵��
         *  @param img ������ͷ/��Ƶ��ȡ����ÿһ֡��Ƶ
         *  @param nFrame ��־�Կ�ʼ��������ǰ֡���ڵڼ�֡���� 0 ��ʼ����
         *  @param direction ��־��������ϳ������³������
         *  @param temp ���ں��������أ���ʵ������
         */
        void featureTrack(cv::Mat& img,ulong nFrame,int direction,int /*temp*/);

        /**
         *  @brief  �� ��־�� ʵ�ֵĻ������ĵ���������͸����㷨��
         */
        void featureTrack(cv::Mat& img,ulong nFrame,int direction);

        /**
         *   ���ڽ����Ĺ켣������ʼ֡����ͬ�Ĺ켣��Ϊһ�飬��������켣�ڲ�ͬ֡����λ����Ϣ��
         */
        void finishedTraInfo(std::string filename = ".\\info.txt");

        /** ���ͬһ���ι켣��ͼƬ��
         *      
         *  ����Ϊÿһ�����ͼƬ����һ���ļ��У����ڴ�Ÿ���ͼƬ
         *  @param  folder �洢���ͼƬ�ĸ��ļ��У�ÿһ��ͼƬ���ļ����ڸ��ļ���֮�´���
         *  @param  videopath ��Ƶ�ļ���ϵͳ·��
         *  @param  off_frame ��Ƶ��ʼ��������֡��
         */
        void images_byteam(std::string folder,std::string videopath,ulong off_frame);

        /** ��ʾ��ÿһ��ͼƬ�����ӵ���Ч�Ĺ켣����������ͼƬ�ϴ�ӡ����Ӧ�����֡�
         *  
         *  @param  folder �洢���ͼƬ�ĸ��ļ��У�ÿһ��ͼƬ���ļ����ڸ��ļ���֮�´���
         *  @param  videopath ��Ƶ�ļ���ϵͳ·��
         *  @param  off_frame ��Ƶ��ʼ��������֡��
         */
        void validFeatures_byFrame(std::string folder,std::string videopath,ulong off_frame);

		/**
		 *	������Ƶ�����еõ�����Ч�����㣬����һ����ɫ�ı���ͼƬ������ЩͼƬ�����ÿһ֡�������㡣
		 *  
		 *  @param folder ��Ž��ͼƬ���ļ���
		 */
		void showFeatures_byFrame(std::string folder);

        /** ���Ѿ���ɵĹ켣������֤
         *
         *  �����ϳ������е����й켣������ʱ����Ӹ�˹�ֲ�����ʼͨ��ѵ���õ������ϳ�����Ҫ��
         *  ƽ��ʱ��ͱ�׼�������켣������ʱ����ѵ���õ��ķֲ���Χƫ��ܴ�����Ϊ�˹켣
         *  Ϊ��Ч�켣
         *
         *  @param mean ѵ�����õ��Ĺ켣��ƽ������ʱ��
         *  @param std_dev ѵ�����õ��Ĺ켣������ʱ��ķ���
         */
        void verify(double mean,double std_dev);

		void verify(double mean,double std_dev,int /*temp*/);

		inline std::multiset<trajectory,_trajectoryCompare>& trajectorys()
		{
			return m_finishTra;
		}
    };
}

#endif