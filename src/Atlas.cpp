/**
  ******************************************************************************
  * @file           : Atlas.cpp
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/13
  ******************************************************************************
 **/

#include "Atlas.h"
#include "Viewer.h"

#include "CameraModels/GeometricCamera.h"
#include "CameraModels/Pinhole.h"
#include "CameraModels/KannalaBrandt8.h"

namespace LoFTR_SLAM
{
    Atlas::Atlas(){
        mpCurrentMap = static_cast<Map*>(NULL);
    }

    Atlas::Atlas(int initKFid): mnLastInitKFidMap(initKFid), mHasViewer(false)
    {
        mpCurrentMap = static_cast<Map*>(NULL);
        CreateNewMap();
    }

    Atlas::~Atlas()
    {
        for(std::set<Map*>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;)
        {
            Map* pMi = *it;

            if(pMi)
            {
                delete pMi;
                pMi = static_cast<Map*>(NULL);

                it = mspMaps.erase(it);
            }
            else
                ++it;

        }
    }

    void Atlas::CreateNewMap()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        cout << "Creation of new map with id: " << Map::nNextId << endl;

        // 保存当前地图集
        if(mpCurrentMap){
            if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
                mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum

            mpCurrentMap->SetStoredMap();
            cout << "Stored map with ID: " << mpCurrentMap->GetId() << endl;

        }
        cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;

        // 新建一个新的空地图
        mpCurrentMap = new Map(mnLastInitKFidMap);
        mpCurrentMap->SetCurrentMap();
        mspMaps.insert(mpCurrentMap);
    }

    void Atlas::ChangeMap(Map* pMap)
    {
        unique_lock<mutex> lock(mMutexAtlas);
        cout << "Change to map with id: " << pMap->GetId() << endl;
        // 保存当前地图
        if(mpCurrentMap){
            mpCurrentMap->SetStoredMap();
        }

        mpCurrentMap = pMap;
        mpCurrentMap->SetCurrentMap();
    }

    void Atlas::SetViewer(Viewer* pViewer)
    {
        mpViewer = pViewer;
        mHasViewer = true;
    }

    void Atlas::AddKeyFrame(KeyFrame* pKF)
    {
        Map* pMapKF = pKF->GetMap();
        pMapKF->AddKeyFrame(pKF);
    }

    void Atlas::AddMapPoint(MapPoint* pMP)
    {
        Map* pMapMP = pMP->GetMap();
        pMapMP->AddMapPoint(pMP);
    }

    GeometricCamera* Atlas::AddCamera(GeometricCamera* pCam)
    {
        //Check if the camera already exists
        bool bAlreadyInMap = false;
        int index_cam = -1;
        for(size_t i=0; i < mvpCameras.size(); ++i)
        {
            GeometricCamera* pCam_i = mvpCameras[i];
            if(!pCam) std::cout << "Not pCam" << std::endl;
            if(!pCam_i) std::cout << "Not pCam_i" << std::endl;
            if(pCam->GetType() != pCam_i->GetType())
                continue;

            if(pCam->GetType() == GeometricCamera::CAM_PINHOLE)
            {
                if(((Pinhole*)pCam_i)->IsEqual(pCam))
                {
                    bAlreadyInMap = true;
                    index_cam = i;
                }
            }
            else if(pCam->GetType() == GeometricCamera::CAM_FISHEYE)
            {
                if(((KannalaBrandt8*)pCam_i)->IsEqual(pCam))
                {
                    bAlreadyInMap = true;
                    index_cam = i;
                }
            }
        }

        if(bAlreadyInMap)
        {
            return mvpCameras[index_cam];
        }
        else{
            mvpCameras.push_back(pCam);
            return pCam;
        }
    }

    std::vector<GeometricCamera*> Atlas::GetAllCameras()
    {
        return mvpCameras;
    }

    void Atlas::SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs)
    {
        unique_lock<mutex> lock(mMutexAtlas);
        mpCurrentMap->SetReferenceMapPoints(vpMPs);
    }

    void Atlas::InformNewBigChange()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        mpCurrentMap->InformNewBigChange();
    }

    int Atlas::GetLastBigChangeIdx()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetLastBigChangeIdx();
    }

    long unsigned int Atlas::MapPointsInMap()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->MapPointsInMap();
    }

    long unsigned Atlas::KeyFramesInMap()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->KeyFramesInMap();
    }

    std::vector<KeyFrame*> Atlas::GetAllKeyFrames()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetAllKeyFrames();
    }

    std::vector<MapPoint*> Atlas::GetAllMapPoints()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetAllMapPoints();
    }

    std::vector<MapPoint*> Atlas::GetReferenceMapPoints()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetReferenceMapPoints();
    }

    vector<Map*> Atlas::GetAllMaps()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        struct compFunctor
        {
            inline bool operator()(Map* elem1 ,Map* elem2)
            {
                return elem1->GetId() < elem2->GetId();
            }
        };
        vector<Map*> vMaps(mspMaps.begin(),mspMaps.end());
        sort(vMaps.begin(), vMaps.end(), compFunctor());
        return vMaps;
    }

    int Atlas::CountMaps()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mspMaps.size();
    }

    void Atlas::clearMap()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        mpCurrentMap->clear();
    }

    void Atlas::clearAtlas()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
        {
            (*it)->clear();
            delete *it;
        }*/
        mspMaps.clear();
        mpCurrentMap = static_cast<Map*>(NULL);
        mnLastInitKFidMap = 0;
    }

    Map* Atlas::GetCurrentMap()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        if(!mpCurrentMap)
            CreateNewMap();
        while(mpCurrentMap->IsBad())
            usleep(3000);

        return mpCurrentMap;
    }

    void Atlas::SetMapBad(Map* pMap)
    {
        mspMaps.erase(pMap);
        pMap->SetBad();

        mspBadMaps.insert(pMap);
    }

    void Atlas::RemoveBadMaps()
    {
        /*for(Map* pMap : mspBadMaps)
        {
            delete pMap;
            pMap = static_cast<Map*>(NULL);
        }*/
        mspBadMaps.clear();
    }

    map<long unsigned int, KeyFrame*> Atlas::GetAtlasKeyframes()
    {
        map<long unsigned int, KeyFrame*> mpIdKFs;
        for(Map* pMap_i : mvpBackupMaps)
        {
            vector<KeyFrame*> vpKFs_Mi = pMap_i->GetAllKeyFrames();

            for(KeyFrame* pKF_j_Mi : vpKFs_Mi)
            {
                mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
            }
        }

        return mpIdKFs;
    }

    long unsigned int Atlas::GetNumLivedKF()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        long unsigned int num = 0;
        for(Map* pMap_i : mspMaps)
        {
            num += pMap_i->GetAllKeyFrames().size();
        }

        return num;
    }

    long unsigned int Atlas::GetNumLivedMP() {
        unique_lock<mutex> lock(mMutexAtlas);
        long unsigned int num = 0;
        for (Map* pMap_i : mspMaps) {
            num += pMap_i->GetAllMapPoints().size();
        }

        return num;
    }

}