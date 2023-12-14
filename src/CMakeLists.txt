add_library( LoFTR_SLAM_from_scratch SHARED
        CameraModels/KannalaBrandt8.cpp
        CameraModels/Pinhole.cpp
        System.cpp
        Converter.cpp
        Settings.cpp
        Tracking.cpp
        FrameDrawer.cpp
        MapDrawer.cpp
        Viewer.cpp
        Frame.cpp
        KeyFrame.cpp
        MapPoint.cpp
        Map.cpp
        Atlas.cpp
        TwoViewReconstruction.cpp
        Random.cpp
        Timestamp.cpp
        Optimizer.cpp
        MLPnPsolver.cpp
        LoFTRMatcher.cpp
        )

target_link_libraries( LoFTR_SLAM_from_scratch  ${THIRD_PARTY_LIBS})
