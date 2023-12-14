/**
  ******************************************************************************
  * @file           : Frame.h
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
 **/



#ifndef FRAME_H
#define FRAME_H

#include<vector>
#include <mutex>

#include <sophus/geometry.hpp>
#include <opencv2/opencv.hpp>
#include "Eigen/Core"
#include "sophus/se3.hpp"

#include "Converter.h"
#include "Settings.h"
#include "MapPoint.h"

namespace LoFTR_SLAM
{
    class MapPoint;
    class KeyFrame;
    class GeometricCamera;

    class Frame
    {
    public:
        Frame();
        /**
         * Copy Constructor.
         * @param frame
         */
        Frame(const Frame &frame);
        /**
         * Constructor for Monocular cameras.
         * @param imGray
         * @param timeStamp
         * @param pCamera
         * @param distCoef
         * @param pPrevF
         */
        Frame(const cv::Mat &imGray, const double &timeStamp, GeometricCamera* pCamera, cv::Mat &distCoef, Frame* pPrevF);

        ~Frame() {};

        // Check, Set and Get Pose
        /**
         * Is the pose set already
         * @return
         */
        bool isSet() const;
        void SetPose(const Sophus::SE3<float> &Tcw);
        inline Sophus::SE3<float> GetPose() const {
            return mTcw;
        }
        inline bool HasPose() const {
            return mbHasPose;
        }

        /**
         * Check if a MapPoint is in the frustum of the camera,
         * and fill variables of the MapPoint to be used by the tracking
         * @param pMP
         * @param viewingCosLimit
         * @return
         */
        bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

        // Get Pose related values;
        inline Eigen::Vector3f GetCameraCenter()
        {
            return mOw;
        }
        inline Eigen::Vector3f GetOw() const {
            return mOw;
        }
        inline Eigen::Matrix3f GetRotationInverse()
        {
            return mRwc;
        }
        inline Eigen::Matrix3f GetRwc() const {
            return mRwc;
        }

        /**
         * 当有其他函数改变 mTcw 的值时，应该对应的修改与位姿有关的变量
         */
        void UpdatePoseMatrices();

        /**
         * 更新特征点，将同一像素坐标的特征点进行合并，并且更改 N, mvpMapPoints, mvbOutlier的大小
         * @param vkps
         * @param vtnMatchIndexes
         * @param flag 表示vkps的下标对应的是vtnMatchIndexes的第一个值还是第二个值
         */
        void UpdateKeyPoint(std::vector<cv::KeyPoint> &vkps, std::vector<std::tuple<int, int>> &vtnMatchIndexes, int flag=1);

        // -------------------------------------------------------------------------------------------------------------

        double mTimeStamp;  // Frame TimeStamp

        // Calibration matrix and OpenCV distortion parameters.
        cv::Mat mK;
        Eigen::Matrix3f mK_;
        static float fx;
        static float fy;
        static float cx;
        static float cy;
        static float invfx;
        static float invfy;
        cv::Mat mDistCoef;

        GeometricCamera* mpCamera;

        int N;  // Number of KeyPoints;

        // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
        std::vector<cv::KeyPoint> mvKeys;
        std::vector<cv::KeyPoint> mvKeysUn;
        // Corresponding stereo coordinate and depth for each keypoint.
        std::vector<MapPoint*> mvpMapPoints;
        // MapPoints associated to keypoints, NULL pointer if no association.
        // Flag to identify outlier associations.
        std::vector<bool> mvbOutlier;

        // Undistorted Image Bounds (computed once).
        static float mnMinX;
        static float mnMaxX;
        static float mnMinY;
        static float mnMaxY;

        // 表示是不是第一次进行计算
        static bool mbInitialComputations;

        // Current and Next Frame id.
        static long unsigned int nNextId;
        long unsigned int mnId;

        cv::Mat mImGray;    // Gray image of current frame

        Frame* mpPrevFrame;    // Pointer to previous frame
        KeyFrame* mpReferenceKF;    // Pointer to reference keyframe
        KeyFrame* mpLastKeyFrame;   // Pointer to last keyframe

    private:
        /**
         * Undistort keypoints given OpenCV distortion parameters.
         * @param start 新插入的mvKeys需要进行undistort
         */
        void UndistortKeyPoints(int start=0);

        // Computes image bounds for the undistorted image (called in the constructor).
        void ComputeImageBounds(const cv::Mat &im);

        // ----------------------------------------------------------
        bool mbHasPose;
        bool mbIsSet;

        // Pose related
        Sophus::SE3<float> mTcw;
        Eigen::Matrix<float,3,3> mRwc;
        Eigen::Matrix<float,3,1> mOw;
        Eigen::Matrix<float,3,3> mRcw;
        Eigen::Matrix<float,3,1> mtcw;
    };

}

#endif //FRAME_H
