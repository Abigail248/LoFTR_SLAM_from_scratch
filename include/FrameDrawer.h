/**
  ******************************************************************************
  * @file           : FrameDrawer.h
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/13
  ******************************************************************************
 **/



#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Atlas.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>
#include <unordered_set>

using namespace std;

namespace LoFTR_SLAM
{
    class Tracking;
    class Viewer;

    class FrameDrawer
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        FrameDrawer(Atlas* pAtlas);

        /**
         * Update info from the last processed frame.
         * @param pTracker
         */
        void Update(Tracking *pTracker);

        /**
         * 准备需要显示的信息，包括图像、特征点、地图、跟踪状态
         * @param imageScale 缩放比例
         * @return
         */
        cv::Mat DrawFrame(float imageScale=1.0f);

        bool both;

    protected:
        void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);
        Atlas* mpAtlas;

        // Info of the frame to be drawn
        cv::Mat mIm;
        int N;
        vector<cv::KeyPoint> mvCurrentKeys;
        //当前帧中的特征点是否在地图中的标记
        //当前帧的特征点在地图中是否出现;后者是表示地图中没有出现,但是在当前帧中是第一次被观测得到的点
        vector<bool> mvbMap, mvbVO;
        bool mbOnlyTracking;
        //当前帧中追踪到的特征点计数
        int mnTracked, mnTrackedVO;
        //参考帧中的特征点
        vector<cv::KeyPoint> mvIniKeys;
        //当前帧特征点和参考帧特征点的匹配关系
        vector<int> mvIniMatches;
        int mState;

        std::mutex mMutex;
        vector<pair<cv::Point2f, cv::Point2f> > mvTracks;

        Frame mCurrentFrame;
        vector<cv::KeyPoint> mvMatchedKeys;
        vector<MapPoint*> mvpMatchedMPs;
        vector<cv::KeyPoint> mvOutlierKeys;
        vector<MapPoint*> mvpOutlierMPs;

        map<long unsigned int, cv::Point2f> mmMatchedInImage;
    };
}

#endif //FRAMEDRAWER_H
