/**
  ******************************************************************************
  * @file           : Tracking.h
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
 **/



#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Atlas.h"
#include "Frame.h"
#include "MapDrawer.h"
#include "System.h"
#include "Settings.h"
#include "LoFTRmatcher.h"
#include "CameraModels/GeometricCamera.h"

#include <mutex>
#include <unordered_set>

namespace LoFTR_SLAM
{

    class Viewer;
    class FrameDrawer;
    class MapDrawer;
    class Atlas;
    class System;
    class Settings;
    class LoFTRmatcher;

    class Tracking
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Tracking(System* pSys, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas, LoFTRmatcher *pMatcher,
                 const string &strSettingPath, const string &strMaskPath , const int sensor, Settings* settings);

        ~Tracking();

        void newParameterLoader(Settings* settings);

        bool ParseCamParamFile(cv::FileStorage &fSettings);

        Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp);

        void SetViewer(Viewer* pViewer);
        void SetStepByStep(bool bSet);
        bool GetStepByStep();

        float GetImageScale();

        void Reset(bool bLocMap = false);
        void ResetActiveMap(bool bLocMap = false);

        void InformOnlyTracking(const bool &flag);

        void CreateMapInAtlas();

        // Tracking states
        enum eTrackingState{
            SYSTEM_NOT_READY=-1,
            NO_IMAGES_YET=0,
            NOT_INITIALIZED=1,
            OK=2,
            RECENTLY_LOST=3,
            LOST=4,
            OK_KLT=5
        };

        // 当前跟踪状态
        eTrackingState mState;
        // 上一帧跟踪的状态
        eTrackingState mLastProcessedState;

        // Input sensor
        int mSensor;

        // Current Frame
        Frame mCurrentFrame;
        Frame mLastFrame;

        // Mask image for all frames
        cv::Mat mImMask;

        // Current frame image;
        cv::Mat mImGray;

        // Initialization Variables (Monocular)
        // Initialization last matches
        std::vector<int> mvIniLastMatches;
        // Initialization matches
        std::vector<int> mvIniMatches;
        // 这个东西目前没有啥用
        std::vector<cv::Point2f> mvbPrevMatched;
        // Initialization 3D point.
        std::vector<cv::Point3f> mvIniP3D;
        // Initialization frame
        Frame mInitialFrame;

        // Lists used to recover the full camera trajectory at the end of the execution.
        // Basically we store the reference keyframe for each frame and its relative transformation
        list<Sophus::SE3f> mlRelativeFramePoses;
        list<KeyFrame*> mlpReferences;
        list<double> mlFrameTimes;
        list<bool> mlbLost;

        // frames with estimated pose
        int mTrackedFr;
        bool mbStep;

        // True if local mapping is deactivated and we are performing only localization
        bool mbOnlyTracking;

        double t0; // timestamp of first read frame

    protected:
        void Track();

        /**
         * Map initialization for monocular
         */
        void MonocularInitialization();

        /**
         * Create Initial Map for Monocular
         */
        void CreateInitialMapMonocular();

        /**
         * 检查上一帧中的地图点是否需要被替换
         */
        void CheckReplacedInLastFrame();

        /**
         * 对参考关键帧的MapPoints进行跟踪（关键帧模式）
         * @return
         */
        bool TrackReferenceKeyFrame();

        /**
         * 参考关键帧的MapPoints进行跟踪（关键帧模式）
         */
        void UpdateLastFrame();

        /**
         * 重定位过程
         * @return
         */
        bool Relocalization();

        void UpdateLocalMap();
        void UpdateLocalPoints();
        void UpdateLocalKeyFrames();

        bool TrackLocalMap();

        /**
          * @brief 判断当前帧是否需要插入关键帧
          *
          * Step 1：纯VO模式下不插入关键帧，如果局部地图被闭环检测使用，则不插入关键帧
          * Step 2：如果距离上一次重定位比较近，或者关键帧数目超出最大限制，不插入关键帧
          * Step 3：得到参考关键帧跟踪到的地图点数量
          * Step 4：决策是否需要插入关键帧
          * @return true         需要
          * @return false        不需要
         */
        bool NeedNewKeyFrame();


        void CreateNewKeyFrame();

        bool mbMapUpdated;

        // In case of performing only localization, this flag is true when there are no matches to
        // points in the map. Still tracking will continue if there are enough matches with temporal points.
        // In that case we are doing visual odometry. The system will try to do relocalization to recover
        // "zero-drift" localization to the map.
        bool mbVO;

        // LoFTR
        LoFTRmatcher* mpLoFTRmatcher;

        KeyFrameDatabase* mpKeyFrameDB;

        // Initalization (only for monocular)
        bool mbReadyToInitializate;
        bool mbSetInit;

        //Local Map
        KeyFrame* mpReferenceKF;
        std::vector<KeyFrame*> mvpLocalKeyFrames;
        std::vector<MapPoint*> mvpLocalMapPoints;


        // System
        System* mpSystem;

        //Drawers
        Viewer* mpViewer;
        FrameDrawer* mpFrameDrawer;
        MapDrawer* mpMapDrawer;
        bool bStepByStep;

        //Atlas
        Atlas* mpAtlas;

        //Calibration matrix
        cv::Mat mK;
        Eigen::Matrix3f mK_;
        cv::Mat mDistCoef;
        float mImageScale;

        bool mInsertKFsLost;

        //New KeyFrame rules (according to fps)
        int mMinFrames;
        int mMaxFrames;

        // Threshold close/far points
        // Points seen as close by the stereo/RGBD sensor are considered reliable
        // and inserted from just one frame. Far points requiere a match in two keyframes.
        float mThDepth;

        //Current matches in frame
        int mnMatchesInliers;

        //Last Frame, KeyFrame and Relocalisation Info
        KeyFrame* mpLastKeyFrame;
        unsigned int mnLastKeyFrameId;
        unsigned int mnLastRelocFrameId;
        double mTimeStampLost;

        unsigned int mnFirstFrameId;
        unsigned int mnInitialFrameId;
        unsigned int mnLastInitFrameId;

        bool mbCreatedMap;

        //Color order (true RGB, false BGR, ignored if grayscale)
        bool mbRGB;

        list<MapPoint*> mlpTemporalPoints;

        //int nMapChangeIndex;

        int mnNumDataset;

        ofstream f_track_stats;

        ofstream f_track_times;
        double mTime_PreIntIMU;
        double mTime_PosePred;
        double mTime_LocalMapTrack;
        double mTime_NewKF_Dec;

        GeometricCamera* mpCamera;

        int initID, lastID;


#ifdef REGISTER_LOOP
        bool Stop();

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;
#endif
    };

}

#endif //TRACKING_H
