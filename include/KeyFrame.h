/**
  ******************************************************************************
  * @file           : KeyFrame.h
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/13
  ******************************************************************************
 **/



#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Frame.h"

#include "CameraModels/GeometricCamera.h"
#include "SerializationUtils.h"

#include <mutex>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>


namespace LoFTR_SLAM {

    class Map;
    class MapPoint;
    class Frame;
    class KeyFrameDatabase;
    class GeometricCamera;

    class KeyFrame {
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & mnId;
            ar & const_cast<long unsigned int &>(mnFrameId);
            ar & const_cast<double &>(mTimeStamp);
            // Scale
            ar & mfScale;
            // Calibration parameters
            ar & const_cast<float &>(fx);
            ar & const_cast<float &>(fy);
            ar & const_cast<float &>(invfx);
            ar & const_cast<float &>(invfy);
            ar & const_cast<float &>(cx);
            ar & const_cast<float &>(cy);
            serializeMatrix(ar, mDistCoef, version);

            // Number of Keypoints
            ar & const_cast<int &>(N);
            // KeyPoints
            serializeVectorKeyPoints<Archive>(ar, mvKeys, version);
            serializeVectorKeyPoints<Archive>(ar, mvKeysUn, version);
            // Pose relative to parent
            serializeSophusSE3<Archive>(ar, mTcp, version);
            // Image bounds and calibration
            ar & const_cast<int &>(mnMinX);
            ar & const_cast<int &>(mnMinY);
            ar & const_cast<int &>(mnMaxX);
            ar & const_cast<int &>(mnMaxY);
            ar & boost::serialization::make_array(mK_.data(), mK_.size());
            // Pose
            serializeSophusSE3<Archive>(ar, mTcw, version);
            // MapPointsId associated to keypoints
            ar & mvBackupMapPointsId;
            // Connected KeyFrameWeight
            ar & mBackupConnectedKeyFrameIdWeights;
            // Bad flags
            ar & mbNotErase;
            ar & mbToBeErased;
            ar & mbBad;

            ar & mnOriginMapId;

            // Camera variables
            ar & mnBackupIdCamera;

        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        KeyFrame();

        KeyFrame(Frame &F, Map *pMap);


        // Pose functions
        void SetPose(const Sophus::SE3f &Tcw);
        Sophus::SE3f GetPose();
        Sophus::SE3f GetPoseInverse();
        Eigen::Vector3f GetCameraCenter();
        Eigen::Matrix3f GetRotation();
        Eigen::Vector3f GetTranslation();

        // Covisibility graph functions
        void AddConnection(KeyFrame* pKF, const int &weight);
        void EraseConnection(KeyFrame* pKF);
        /**
         * 在没有执行这个函数前，关键帧只和MapPoints之间有连接关系，这个函数可以更新关键帧之间的连接关系
         *
         * @param upParent
         */
        void UpdateConnections(bool upParent=true);
        void UpdateBestCovisibles();
        std::set<KeyFrame *> GetConnectedKeyFrames();
        std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
        std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
        std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
        int GetWeight(KeyFrame* pKF);

        // Spanning tree functions
        void AddChild(KeyFrame* pKF);
        void EraseChild(KeyFrame* pKF);
        void ChangeParent(KeyFrame* pKF);
        std::set<KeyFrame*> GetChilds();
        KeyFrame* GetParent();
        bool hasChild(KeyFrame* pKF);
        void SetFirstConnection(bool bFirst);

        // MapPoint observation functions
        int GetNumberMPs();
        void AddMapPoint(MapPoint* pMP, const size_t &idx);
        void EraseMapPointMatch(const int &idx);
        void EraseMapPointMatch(MapPoint* pMP);
        void ReplaceMapPointMatch(const int &idx, MapPoint* pMP);
        std::set<MapPoint*> GetMapPoints();
        std::vector<MapPoint*> GetMapPointMatches();
        int TrackedMapPoints(const int &minObs);
        MapPoint* GetMapPoint(const size_t &idx);

        // Image
        bool IsInImage(const float &x, const float &y) const;

        // Enable/Disable bad flag changes
        void SetNotErase();
        void SetErase();

        // Set/check bad flag
        /**
         * 真正的执行删除关键帧的操作，需要删除的是该关键帧和其他所有帧、地图点之间的连接关系
         */
        void SetBadFlag();
        bool isBad();

        // Compute Scene Depth (q=2 median). Used in monocular.
        float ComputeSceneMedianDepth(const int q);

        static bool weightComp( int a, int b){
            return a>b;
        }

        static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
            return pKF1->mnId<pKF2->mnId;
        }

        Map* GetMap();
        void UpdateMap(Map* pMap);


        static long unsigned int nNextId;
        long unsigned int mnId;
        const long unsigned int mnFrameId;

        const double  mTimeStamp;

        // Variables used by the tracking
        long unsigned int mnTrackReferenceForFrame;     // 记录它
        long unsigned int mnFuseTargetForKF;    // 标记在局部建图线程中,和哪个关键帧进行融合的操作

        //Number of optimizations by BA(amount of iterations in BA)
        long unsigned int mnNumberOfOpt;

        float mfScale;

        // Variables used by the keyframe database
        long unsigned int mnLoopQuery;
        int mnLoopWords;
        float mLoopScore;
        long unsigned int mnRelocQuery;
        int mnRelocWords;
        float mRelocScore;
        long unsigned int mnMergeQuery;
        int mnMergeWords;
        float mMergeScore;
        long unsigned int mnPlaceRecognitionQuery;
        int mnPlaceRecognitionWords;
        float mPlaceRecognitionScore;

        bool mbCurrentPlaceRecognition;

        // Calibration parameters
        const float fx, fy, cx, cy, invfx, invfy;
        cv::Mat mDistCoef;

        // Number of KeyPoints
        const int N;

        // KeyPoints, stereo coordinate and descriptors (all associated by an index)
        const std::vector<cv::KeyPoint> mvKeys;
        const std::vector<cv::KeyPoint> mvKeysUn;

        // Pose relative to parent (this is computed when bad flag is activated)
        Sophus::SE3f mTcp;

        // Image bounds and calibration
        const int mnMinX;
        const int mnMinY;
        const int mnMaxX;
        const int mnMaxY;

        unsigned int mnOriginMapId;

        GeometricCamera* mpCamera;

        cv::Mat mImGray;

    protected:
        // Sophus poses
        Sophus::SE3<float> mTcw;
        Eigen::Matrix3f mRcw;
        Sophus::SE3<float> mTwc;
        Eigen::Matrix3f mRwc;

        // MapPoints associated to keypoints
        std::vector<MapPoint*> mvpMapPoints;
        // For save relation without pointer, this is necessary for save/load function
        std::vector<long long int> mvBackupMapPointsId;

        std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
        std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
        std::vector<int> mvOrderedWeights;
        // For save relation without pointer, this is necessary for save/load function
        std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;

        // Spanning Tree Edges
        bool mbFirstConnection;
        KeyFrame* mpParent;
        std::set<KeyFrame*> mspChildrens;

        // Bad flags
        bool mbNotErase;
        bool mbToBeErased;
        bool mbBad;

        Map* mpMap;

        // Backup for Cameras
        unsigned int mnBackupIdCamera;

        // Calibration
        Eigen::Matrix3f mK_;

        // Mutex
        std::mutex mMutexPose; // for pose, velocity and biases
        std::mutex mMutexConnections;
        std::mutex mMutexFeatures;
        std::mutex mMutexMap;

    };
}


#endif //KEYFRAME_H
