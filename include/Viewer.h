/**
  ******************************************************************************
  * @file           : Viewer.h
  * @author         : abigail
  * @brief          : 可视化查看器的声明
  * @attention      : None
  * @date           : 2023/12/13
  ******************************************************************************
 **/



#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "Settings.h"

#include <mutex>

namespace LoFTR_SLAM
{
    class Tracking;
    class FrameDrawer;
    class MapDrawer;
    class System;
    class Settings;

    class Viewer
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath, Settings* settings);

        void newParameterLoader(Settings* settings);

        void Run();
        bool isStopped();
        void RequestStop();
        void Release();

        bool both;

    private:
        bool ParseViewerParamFile(cv::FileStorage &fSettings);

        bool Stop();

        System* mpSystem;
        FrameDrawer* mpFrameDrawer;
        MapDrawer* mpMapDrawer;
        Tracking* mpTracker;

        // 1/fps in ms
        double mT;
        float mImageWidth, mImageHeight;
        float mImageViewerScale;

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        std::mutex mMutexStop;

        bool mbStopTrack;
    };
}

#endif //VIEWER_H
