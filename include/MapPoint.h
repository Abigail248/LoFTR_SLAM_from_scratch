/**
  ******************************************************************************
  * @file           : MapPoint.h
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/13
  ******************************************************************************
 **/



#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Converter.h"

#include "SerializationUtils.h"

#include <opencv2/core/core.hpp>
#include <mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>

namespace LoFTR_SLAM
{
    class KeyFrame;
    class Map;
    class Frame;

    class MapPoint
    {
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & mnId;
            ar & mnFirstKFid;
            ar & mnFirstFrame;
            ar & nObs;
            // Variables used by the tracking
            ar & mTrackProjX;
            ar & mTrackProjY;
            ar & mTrackDepth;
            ar & mbTrackInView;
            ar & mTrackViewCos;
            ar & mnTrackReferenceForFrame;
            ar & mnLastFrameSeen;

            // Protected variables
            ar & boost::serialization::make_array(mWorldPos.data(), mWorldPos.size());
            ar & boost::serialization::make_array(mNormalVector.data(), mNormalVector.size());
            ar & mObservations;
            ar & mnVisible;
            ar & mnFound;

            ar & mbBad;

        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        MapPoint();

        MapPoint(const Eigen::Vector3f &Pos, KeyFrame* pRefKF, Map* pMap);
        MapPoint(const Eigen::Vector3f &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

        // Set, Get World Pose
        void SetWorldPos(const Eigen::Vector3f &Pos);
        Eigen::Vector3f GetWorldPos();

        // Set, Get and Update Normal (观测方向)
        void SetNormalVector(const Eigen::Vector3f& normal);
        Eigen::Vector3f GetNormal();
        /**
 * 更新平均观测方向
 * 由于一个MapPoint会被许多相机观测到，因此在插入关键帧后，需要更新相应变量
 * 创建新的关键帧的时候会调用
 */
        void UpdateNormal();

        int Observations();
        std::map<KeyFrame*, int> GetObservations();
        /**
         * 记录哪些 KeyFrame 的那个特征点能观测到该地图点
         * 并增加观测的相机数目nObs
         * @param pKF
         * @param idx MapPoint在KeyFrame中的索引
         */
        void AddObservation(KeyFrame* pKF,int idx);
        void EraseObservation(KeyFrame* pKF);
        void PrintObservations();

        // If MapPoint in KeyFrame, get reference index in KeyFrame
        int GetIndexInKeyFrame(KeyFrame* pKF);
        bool IsInKeyFrame(KeyFrame* pKF);

        /**
         * 获得首次观测到该地图点的关键帧
         * @return KeyFrame
         */
        KeyFrame* GetReferenceKeyFrame();

        // Get and Set Bad MapPoint Flag
        /**
         * 没有经过 MapPointCulling 检测的MapPoints, 认为是坏掉的点
         * @return
         */
        bool isBad();
        void SetBadFlag();

        // Set and Get MapPoint replaced this MapPoint
        /**
         * 替换地图关键点（合并也可以）
         * @param pMP 用该地图点来替换当前地图点
         */
        void Replace(MapPoint* pMP);
        MapPoint* GetReplaced();

        // Get and Set Visible and Found times
        void IncreaseVisible(int n=1);
        void IncreaseFound(int n=1);
        float GetFoundRatio();
        inline int GetFound(){
            return mnFound;
        }

        Map* GetMap();
        void UpdateMap(Map* pMap);

        // -------------------------------------------------------------------
        long unsigned int mnId;
        static long unsigned int nNextId;
        long int mnFirstKFid;   // 第一次观测该地图点的关键帧的编号
        long int mnFirstFrame;      // 第一次观测到该地图点的帧的编号
        int nObs;       // 该点被观测到的次数

        // Variables used by the tracking
        float mTrackProjX;
        float mTrackProjY;
        float mTrackDepth;
        bool mbTrackInView;     // mbTrackInView是决定一个地图点是否进行重投影的标志
        float mTrackViewCos;
        // TrackLocalMap - UpdateLocalPoints 中防止将MapPoints重复添加至mvpLocalMapPoints的标记
        long unsigned int mnTrackReferenceForFrame;
        // TrackLocalMap - SearchLocalPoints 中决定是否进行isInFrustum判断的变量
        // NOTICE mnLastFrameSeen==mCurrentFrame.mnId的点有几种：
        // a 已经和当前帧经过匹配（TrackReferenceKeyFrame，TrackWithMotionModel）但在优化过程中认为是外点
        // b 已经和当前帧经过匹配且为内点，这类点也不需要再进行投影
        long unsigned int mnLastFrameSeen;

        // Fopr inverse depth optimization
        double mInvDepth;
        double mInitU;
        double mInitV;
        KeyFrame* mpHostKF;

        static std::mutex mGlobalMutex;

        unsigned int mnOriginMapId;

    protected:
        // Position in absolute coordinates
        Eigen::Vector3f mWorldPos;

        // Keyframes observing the point and associated index in keyframe
        std::map<KeyFrame*,int> mObservations;

        // Visible 表示该MapPoint在某些帧的视野范围之内，通过Frame::isInFrustum()函数判断
        // 该MapPoint被这些帧观测到，但并不一定能和这些帧的特征点匹配上
        int mnVisible;
        int mnFound;
        bool mbBad;

        MapPoint* mpReplaced;

        // Mean viewing direction
        Eigen::Vector3f mNormalVector;

        // Reference KeyFrame
        KeyFrame* mpRefKF;

        Map* mpMap;
        // Mutex
        std::mutex mMutexPos;
        std::mutex mMutexFeatures;
        std::mutex mMutexMap;


    };
}

#endif //MAPPOINT_H
