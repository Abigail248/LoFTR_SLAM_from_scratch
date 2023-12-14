/**
  ******************************************************************************
  * @file           : MapPoint.cpp
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/13
  ******************************************************************************
 **/

#include "MapPoint.h"

#include <mutex>

namespace LoFTR_SLAM
{
    long unsigned int MapPoint::nNextId=0;
    mutex MapPoint::mGlobalMutex;

    MapPoint::MapPoint():
            mnFirstKFid(0), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
            mnVisible(1), mnFound(1), mbBad(false), mpReplaced(static_cast<MapPoint*>(NULL))
    {
        mpReplaced = static_cast<MapPoint*>(NULL);
    }

    MapPoint::MapPoint(const Eigen::Vector3f &Pos, KeyFrame *pRefKF, Map* pMap):
            mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
            mnLastFrameSeen(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false), mpReplaced(static_cast<MapPoint*>(NULL)),
            mpMap(pMap), mnOriginMapId(pMap->GetId())
    {
        SetWorldPos(Pos);

        mNormalVector.setZero();

        mbTrackInView = false;

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId=nNextId++;
    }

    MapPoint::MapPoint(const Eigen::Vector3f &Pos, Map* pMap, Frame* pFrame, const int &idxF):
            mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
            mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
            mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), mnOriginMapId(pMap->GetId())
    {
        SetWorldPos(Pos);

        Eigen::Vector3f Ow;

        Ow = pFrame->GetCameraCenter();

        mNormalVector = mWorldPos - Ow;
        mNormalVector = mNormalVector / mNormalVector.norm();

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId=nNextId++;
    }

    void MapPoint::SetWorldPos(const Eigen::Vector3f &Pos) {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        mWorldPos = Pos;
    }

    Eigen::Vector3f MapPoint::GetWorldPos() {
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos;
    }

    Eigen::Vector3f MapPoint::GetNormal() {
        unique_lock<mutex> lock(mMutexPos);
        return mNormalVector;
    }

    KeyFrame* MapPoint::GetReferenceKeyFrame()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mpRefKF;
    }

    void MapPoint::AddObservation(KeyFrame* pKF, int idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);

        mObservations[pKF]=idx;

        nObs++;
    }

    void MapPoint::EraseObservation(KeyFrame* pKF)
    {
        bool bBad=false;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if(mObservations.count(pKF))
            {
                int index = mObservations[pKF];

                if(index != -1){
                    nObs--;
                }

                mObservations.erase(pKF);

                // 如果该关键帧刚好是第一个观测到该点的关键帧，那么将第一次观测到该点的关键帧换成队列中的第一个
                if(mpRefKF==pKF)
                    mpRefKF=mObservations.begin()->first;

                // If only 2 observations or less, discard point
                if(nObs<=2)
                    bBad=true;
            }
        }

        if(bBad)
            SetBadFlag();
    }

    std::map<KeyFrame*, int>  MapPoint::GetObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    }

    int MapPoint::Observations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return nObs;
    }

    void MapPoint::SetBadFlag()
    {
        map<KeyFrame*, int> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            mbBad=true;
            obs = mObservations;
            mObservations.clear();
        }

        // 因为MapPoint 和 KeyFrame 是相互标记的，所以在要删掉在关键帧的记录
        for(map<KeyFrame*, int>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
        {
            KeyFrame* pKF = mit->first;
            int index = mit -> second;
            if(index != -1){
                pKF->EraseMapPointMatch(index);
            }
        }

        mpMap->EraseMapPoint(this);
    }

    MapPoint* MapPoint::GetReplaced()
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mpReplaced;
    }

    void MapPoint::Replace(MapPoint* pMP)
    {
        if(pMP->mnId==this->mnId)
            return;

        int nvisible, nfound;

        // 清除该MapPoint
        map<KeyFrame*,int> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            obs=mObservations;
            mObservations.clear();
            mbBad=true;
            nvisible = mnVisible;
            nfound = mnFound;
            mpReplaced = pMP;
        }

        for(map<KeyFrame*,int>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
        {
            // Replace measurement in keyframe
            KeyFrame* pKF = mit->first;

            int index = mit -> second;

            if(!pMP->IsInKeyFrame(pKF))
            {
                if(index != -1){
                    pKF->ReplaceMapPointMatch(index, pMP);
                    pMP->AddObservation(pKF,index);
                }
            }
            else
            {
                if(index != -1){
                    pKF->EraseMapPointMatch(index);
                }
            }
        }
        pMP->IncreaseFound(nfound);
        pMP->IncreaseVisible(nvisible);;

        mpMap->EraseMapPoint(this);
    }

    bool MapPoint::isBad()
    {
        unique_lock<mutex> lock1(mMutexFeatures,std::defer_lock);
        unique_lock<mutex> lock2(mMutexPos,std::defer_lock);
        lock(lock1, lock2);

        return mbBad;
    }

    void MapPoint::IncreaseVisible(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnVisible+=n;
    }

    void MapPoint::IncreaseFound(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnFound+=n;
    }

    float MapPoint::GetFoundRatio()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return static_cast<float>(mnFound)/mnVisible;
    }

    int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
            return mObservations[pKF];
        else
            return -1;
    }

    bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return (mObservations.count(pKF));
    }

    void MapPoint::UpdateNormal()
    {
        //  Step 1 获得该地图点的相关信息
        map<KeyFrame*, int> observations;
        KeyFrame* pRefKF;
        Eigen::Vector3f Pos;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            if(mbBad)
                return;
            observations = mObservations;  // 得观测到该地图点的所有关键帧
            pRefKF = mpRefKF;   // 观测到该点的参考关键帧（第一次创建时的关键帧）
            Pos = mWorldPos;       // 地图点在世界坐标系中的位置
        }

        if(observations.empty())
            return;

        // Step 2 计算该地图点的法线方向，也就是朝向等信息。
        // 能观测到该地图点的所有关键帧，对该点的观测方向归一化为单位向量，然后进行求和得到该地图点的朝向
        // 初始值为0向量，累加为归一化向量，最后除以总数n
        Eigen::Vector3f normal;
        normal.setZero();
        int n=0;
        for(map<KeyFrame*,int >::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKF = mit->first;

            int index = mit -> second;

            if(index != -1){
                Eigen::Vector3f Owi = pKF->GetCameraCenter();
                Eigen::Vector3f normali = Pos - Owi;
                // 计算当前观测方向
                normal = normal + normali / normali.norm();
                n++;
            }
        }

        // 参考关键帧相机指定地图点的向量（第一次创建改地图点的关键帧）
        Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();

        int index = observations[pRefKF];

        {
            unique_lock<mutex> lock3(mMutexPos);
            mNormalVector = normal/n;
        }
    }

    void MapPoint::SetNormalVector(const Eigen::Vector3f& normal)
    {
        unique_lock<mutex> lock3(mMutexPos);
        mNormalVector = normal;
    }

    void MapPoint::PrintObservations()
    {
        cout << "MP_OBS: MP " << mnId << endl;
        for(map<KeyFrame*,int>::iterator mit=mObservations.begin(), mend=mObservations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            cout << "--OBS in KF " << pKFi->mnId << " in map " << pKFi->GetMap()->GetId() << endl;
        }
    }

    Map* MapPoint::GetMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void MapPoint::UpdateMap(Map* pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }
}
