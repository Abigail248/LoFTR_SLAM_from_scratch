/**
  ******************************************************************************
  * @file           : System.h
  * @author         : abigail
  * @brief          : 创建一个初始系统,这个头文件定义了SLAM的主线程（或者称之为系统）结构，其他的各个模块都是由这里开始被调用的。
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
 **/

#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <thread>
#include <opencv2/core/core.hpp>

#include <sophus/sim3.hpp>
#include <sophus/se3.hpp>
#include <mutex>

#include "Tracking.h"
#include "Map.h"
#include "LoFTRMatcher.h"
#include "Atlas.h"
#include "Settings.h"

namespace LoFTR_SLAM
{
    class Viewer;
    class FrameDrawer;
    class MapDrawer;
    class Atlas;
    class Map;
    class Tracking;
    class LoFTRMatcher;

    class Verbose
    {
    public:
        enum eLevel
        {
            VERBOSITY_QUIET=0,
            VERBOSITY_NORMAL=1,
            VERBOSITY_VERBOSE=2,
            VERBOSITY_VERY_VERBOSE=3,
            VERBOSITY_DEBUG=4
        };

        static eLevel th;

    public:
        static void PrintMess(std::string str, eLevel lev)
        {
            if(lev <= th)
                std::cout << str << std::endl;
        }

        static void SetTh(eLevel _th)
        {
            th = _th;
        }
    };

    class System
    {
    public:
        // Input Sensor
        enum eSensor
        {
            MONOCULAR=0,
            MONOCULAR_EMT = 1
        };

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * 构造函数，用来初始化整个系统。
         * @param strMaskFile 指定整套数据的mask图像的路径
         * @param strSettingsFile 指定配置文件的路径
         * @param strLoFTRModelFile 指定LoFTR模型的路径
         * @param sensor 指定所使用的传感器类型
         * @param bUseViewer 指定是否使用可视化界面
         * @param initFr 指定初始帧
         */
        System(const std::string &strMaskFile, const std::string &strSettingsFile,
               const std::string &strLoFTRModelFile, const eSensor sensor, const bool bUseViewer = true,
               const int initFr = 0);

        // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Returns the camera pose (empty if tracking fails).
        /**
         * 进行单目相机的运动跟踪
         * @param im 图像 RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
         * @param timestamp 时间戳
         * @return 追踪成功返回估计的相机位姿，如果追踪失败则返回NULL
         */
        Sophus::SE3f TrackMonocular(const cv::Mat &im, const double &timestamp);

        /**
         * 复位 系统 clear Atlas or the active map
         */
        void Reset();
        void ResetActiveMap();

        /**
         * All threads will be requested to finish.
         * It waits until all threads have finished.
         * This function must be called before saving the trajectory.
         */
        void Shutdown();
        bool isShutDown();


        /**
         * For debuging! Information from most recent processed frame, You can call this right after TrackMonocular
         * @return
         */
        int GetTrackingState();
        std::vector<MapPoint*> GetTrackedMapPoints();
        std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

        /**
         * For debugging
         */
        bool isLost();
        bool isFinished();

        /**
         * Get image scale.
         * @return
         */
        float GetImageScale();

#ifdef REGISTER_TIMES
        void InsertRectTime(double& time);
    void InsertResizeTime(double& time);
    void InsertTrackTime(double& time);
#endif

    private:


        std::string CalculateCheckSum(std::string filename, int type);

        // Input sensor
        eSensor mSensor;

        // LoFTR matcher used for feature matching
        LoFTRmatcher *mpLoFTRMatcher;

        // Map structure that stores the pointers to all KeyFrames and MapPoints.
        Atlas* mpAtlas;

        // Tracker. It receives a frame and computes the associated camera pose.
        // It also decides when to insert a new keyframe, create some new MapPoints and
        // performs relocalization if tracking fails.
        Tracking* mpTracker;

        // The viewer draws the map and the current camera pose. It uses Pangolin.
        Viewer* mpViewer;

        FrameDrawer* mpFrameDrawer;

        MapDrawer* mpMapDrawer;

        // System threads Viewer.
        // The Tracking thread "lives" in the main execution thread that creates the System object.
        std::thread* mptViewer;

        // Reset flag
        std::mutex mMutexReset;
        bool mbReset;
        bool mbResetActiveMap;

        // Change mode flags
        std::mutex mMutexMode;
        bool mbActivateLocalizationMode;
        bool mbDeactivateLocalizationMode;

        // Shutdown flag
        bool mbShutDown;

        // Tracking state
        int mTrackingState;
        std::vector<MapPoint*> mTrackedMapPoints;
        std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
        std::mutex mMutexState;

        string mStrLoFTRModelFile;

        Settings* settings_;
    };

}


#endif //SYSTEM_H
