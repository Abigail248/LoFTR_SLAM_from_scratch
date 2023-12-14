/**
  ******************************************************************************
  * @file           : Map.cpp
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/13
  ******************************************************************************
 **/

#include "Map.h"

#include<mutex>

namespace LoFTR_SLAM
{
    long unsigned int Map::nNextId=0;

    Map::Map(): mnMaxKFid(0), mnBigChangeIdx(0), mnMapChange(0), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
    mbFail(false), mIsInUse(false), mHasTumbnail(false), mbBad(false), mnMapChangeNotified(0)
    {
        mnId=nNextId++;
        mThumbnail = static_cast<GLubyte*>(NULL);
    }


    Map::Map(int initKFid): mnInitKFid(initKFid), mnMaxKFid(initKFid), mnBigChangeIdx(0), mIsInUse(false),
    mHasTumbnail(false), mbBad(false), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
    mnMapChange(0), mbFail(false), mnMapChangeNotified(0)
    {
        mnId=nNextId++;
        mThumbnail = static_cast<GLubyte*>(NULL);
    }

    Map::~Map()
    {
        //TODO: erase all points from memory
        mspMapPoints.clear();

        //TODO: erase all keyframes from memory
        mspKeyFrames.clear();

        if(mThumbnail)
            delete mThumbnail;
        mThumbnail = static_cast<GLubyte*>(NULL);

        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
    }

    void Map::AddKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexMap);
        if(mspKeyFrames.empty()){
            cout << "First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFid << endl;
            mnInitKFid = pKF->mnId;
            mpKFinitial = pKF;
            mpKFlowerID = pKF;
        }
        mspKeyFrames.insert(pKF);
        if(pKF->mnId>mnMaxKFid)
        {
            mnMaxKFid=pKF->mnId;
        }
        if(pKF->mnId<mpKFlowerID->mnId)
        {
            mpKFlowerID = pKF;
        }
    }

    void Map::AddMapPoint(MapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
    }

    void Map::EraseMapPoint(MapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP);
    }

    void Map::EraseKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);

        // 更新第一图中最后一张关键帧
        if(mspKeyFrames.size()>0)
        {
            if(pKF->mnId == mpKFlowerID->mnId)
            {
                vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
                sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
                mpKFlowerID = vpKFs[0];
            }
        }
        else
        {
            mpKFlowerID = 0;
        }
    }

    void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
    {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }

    void Map::InformNewBigChange()
    {
        unique_lock<mutex> lock(mMutexMap);
        mnBigChangeIdx++;
    }

    int Map::GetLastBigChangeIdx()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnBigChangeIdx;
    }

    vector<KeyFrame*> Map::GetAllKeyFrames()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
    }

    vector<MapPoint*> Map::GetAllMapPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
    }

    long unsigned int Map::MapPointsInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }

    long unsigned int Map::KeyFramesInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

    vector<MapPoint*> Map::GetReferenceMapPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }

    long unsigned int Map::GetId()
    {
        return mnId;
    }

    long unsigned int Map::GetInitKFid()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnInitKFid;
    }

    void Map::SetInitKFid(long unsigned int initKFif)
    {
        unique_lock<mutex> lock(mMutexMap);
        mnInitKFid = initKFif;
    }

    long unsigned int Map::GetMaxKFid()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    KeyFrame* Map::GetOriginKF()
    {
        return mpKFinitial;
    }

    void Map::SetCurrentMap()
    {
        mIsInUse = true;
    }

    void Map::SetStoredMap()
    {
        mIsInUse = false;
    }

    void Map::clear()
    {

        for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        {
            KeyFrame* pKF = *sit;
            pKF->UpdateMap(static_cast<Map*>(NULL));
        }

        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = mnInitKFid;
        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
    }

    bool Map::IsInUse()
    {
        return mIsInUse;
    }

    void Map::SetBad()
    {
        mbBad = true;
    }

    bool Map::IsBad()
    {
        return mbBad;
    }

    void Map::ChangeId(long unsigned int nId)
    {
        mnId = nId;
    }

    unsigned int Map::GetLowerKFID()
    {
        unique_lock<mutex> lock(mMutexMap);
        if (mpKFlowerID) {
            return mpKFlowerID->mnId;
        }
        return 0;
    }

    int Map::GetMapChangeIndex()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnMapChange;
    }

    void Map::IncreaseChangeIndex()
    {
        unique_lock<mutex> lock(mMutexMap);
        mnMapChange++;
    }

    int Map::GetLastMapChange()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnMapChangeNotified;
    }

    void Map::SetLastMapChange(int currentChangeId)
    {
        unique_lock<mutex> lock(mMutexMap);
        mnMapChangeNotified = currentChangeId;
    }

}