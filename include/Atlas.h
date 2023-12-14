/**
  ******************************************************************************
  * @file           : Atlas.h
  * @author         : abigail
  * @brief          : Atlas相当于一个子系统，它保存了许多SLAM所创建的子地图。
  *                   所有子地图可以分成两类：Active Map(跟踪定位所使用的地图)，
  *                   Non-active Map(之前保留下来的地图)。
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
 **/



#ifndef ATLAS_H
#define ATLAS_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "CameraModels/GeometricCamera.h"
#include "CameraModels/Pinhole.h"
#include "CameraModels/KannalaBrandt8.h"

#include <set>
#include <mutex>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>

using namespace std;

namespace LoFTR_SLAM
{
    class Viewer;
    class Map;
    class MapPoint;
    class KeyFrame;
    class Frame;
    class KannalaBrandt8;
    class Pinhole;

    class Atlas
    {
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar.template register_type<Pinhole>();
            ar.template register_type<KannalaBrandt8>();

            // Save/load a set structure, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
            //ar & mspMaps;
            ar & mvpBackupMaps;
            ar & mvpCameras;
            // Need to save/load the static Id from Frame, KeyFrame, MapPoint and Map
            ar & Map::nNextId;
            ar & Frame::nNextId;
            ar & KeyFrame::nNextId;
            ar & MapPoint::nNextId;
            ar & GeometricCamera::nNextId;
            ar & mnLastInitKFidMap;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Atlas();
        Atlas(int initKFid); // When its initialization the first map is created
        ~Atlas();

        void CreateNewMap();
        void ChangeMap(Map* pMap);

        unsigned long int GetLastInitKFid();

        void SetViewer(Viewer* pViewer);

        // Method for change components in the current map
        void AddKeyFrame(KeyFrame* pKF);
        void AddMapPoint(MapPoint* pMP);
//        void EraseMapPoint(MapPoint* pMP);
//        void EraseKeyFrame(KeyFrame* pKF);

        GeometricCamera* AddCamera(GeometricCamera* pCam);
        std::vector<GeometricCamera*> GetAllCameras();

        /* All methods without Map pointer work on current map */
        void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
        void InformNewBigChange();
        int GetLastBigChangeIdx();

        long unsigned int MapPointsInMap();
        long unsigned KeyFramesInMap();

        // Method for get data in current map
        std::vector<KeyFrame*> GetAllKeyFrames();
        std::vector<MapPoint*> GetAllMapPoints();
        std::vector<MapPoint*> GetReferenceMapPoints();

        vector<Map*> GetAllMaps();

        int CountMaps();

        void clearMap();

        void clearAtlas();

        Map* GetCurrentMap();

        void SetMapBad(Map* pMap);
        void RemoveBadMaps();

        // Function for garantee the correction of serialization of this object
        void PreSave();
        void PostLoad();

        map<long unsigned int, KeyFrame*> GetAtlasKeyframes();

        long unsigned int GetNumLivedKF();

        long unsigned int GetNumLivedMP();

    protected:

        std::set<Map*> mspMaps;
        std::set<Map*> mspBadMaps;
        // Its necessary change the container from set to vector because libboost 1.58 and Ubuntu 16.04 have an error with this cointainer
        std::vector<Map*> mvpBackupMaps;

        Map* mpCurrentMap;

        std::vector<GeometricCamera*> mvpCameras;

        unsigned long int mnLastInitKFidMap;

        Viewer* mpViewer;
        bool mHasViewer;

        // Mutex
        std::mutex mMutexAtlas;


    }; // class Atlas

} // namespace LoFTR_SLAM


#endif //ATLAS_H
