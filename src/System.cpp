/**
  ******************************************************************************
  * @file           : System.cpp
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
 **/

#include "System.h"
#include "Converter.h"
#include "Settings.h"


#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

using namespace std;

namespace LoFTR_SLAM
{
    System::System(const std::string &strMaskFile, const std::string &strSettingsFile,
                   const std::string &strLoFTRModelFile, const LoFTR_SLAM::System::eSensor sensor,
                   const bool bUseViewer, const int initFr)
    {
        // Output message
        std::cout << "Input sensor was set to: ";

        if(mSensor==MONOCULAR)
            cout << "Monocular" << endl;
        else if(mSensor==MONOCULAR_EMT)
            cout << "RGB-D-Inertial" << endl;

        //Check settings file
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }

        cv::FileNode node = fsSettings["File.version"];
        if(!node.empty() && node.isString() && node.string() == "1.0"){
            settings_ = new Settings(strSettingsFile,mSensor);
            cout << (*settings_) << endl;
        }
        else{
            settings_ = nullptr;
        }


        mStrLoFTRModelFile = strLoFTRModelFile;

        bool loadedAtlas = false;

        // Load LoFTR Onnx Model
        cout << endl << "Load LoFTR Onnx Model. This could take a while..." << endl;
        mpLoFTRMatcher = new LoFTRmatcher(strLoFTRModelFile);
        cout << "LoFTR model loaded!" << endl << endl;

        // Create the Atlas
        cout << "Initialization of Atlas from scratch " << endl;
        mpAtlas = new Atlas(0);

        //Create Drawers. These are used by the Viewer
        mpFrameDrawer = new FrameDrawer(mpAtlas);
        mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);

        //Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = new Tracking(this, mpFrameDrawer, mpMapDrawer, mpAtlas, mpLoFTRMatcher,
                                 strSettingsFile, strMaskFile, mSensor, settings_);


        //Initialize the Viewer thread and launch
        if(bUseViewer)
            //if(false) // TODO
        {
            mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile,settings_);
            // 用子线程跑
            mptViewer = new thread(&Viewer::Run, mpViewer);
            mpTracker->SetViewer(mpViewer);
            mpViewer->both = mpFrameDrawer->both;
        }

        // Fix verbosity
        Verbose::SetTh(Verbose::VERBOSITY_QUIET);
    }

    Sophus::SE3f System::TrackMonocular(const cv::Mat &im, const double &timestamp) {

        {
            unique_lock<mutex> lock(mMutexReset);
            if(mbShutDown)
                return Sophus::SE3f();
        }

        // 判断输入是否为Monocular or IMU+Monocular
        if(mSensor!=MONOCULAR && mSensor!=MONOCULAR_EMT)
        {
            cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << endl;
            exit(-1);
        }

        cv::Mat imToFeed = im.clone();
        // Resize image
        if(settings_ && settings_->needToResize()){
            cv::Mat resizedIm;
            cv::resize(im,resizedIm,settings_->newImSize());
            imToFeed = resizedIm;
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if(mbActivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if(mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if(mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
                mbResetActiveMap = false;
            }
            else if(mbResetActiveMap)
            {
                cout << "SYSTEM-> Reseting active map in monocular case" << endl;
                mpTracker->ResetActiveMap();
                mbResetActiveMap = false;
            }
        }

        // 根据当前帧获得相机的位姿
        Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(imToFeed,timestamp);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
        return Tcw;
    }

    float System::GetImageScale() {
        return mpTracker->GetImageScale();
    }

    void System::Reset()
    {
        unique_lock<mutex> lock(mMutexReset);
        mbReset = true;
    }

    void System::ResetActiveMap()
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetActiveMap = true;
    }

    void System::Shutdown()
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbShutDown = true;
        }

        cout << "Shutdown" << endl;
        /*if(mpViewer)
        {
            mpViewer->RequestFinish();
            while(!mpViewer->isFinished())
                usleep(5000);
        }*/

        // Wait until all thread have effectively stopped
        /*while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
        {
            if(!mpLocalMapper->isFinished())
                cout << "mpLocalMapper is not finished" << endl;*/
        /*if(!mpLoopCloser->isFinished())
            cout << "mpLoopCloser is not finished" << endl;
        if(mpLoopCloser->isRunningGBA()){
            cout << "mpLoopCloser is running GBA" << endl;
            cout << "break anyway..." << endl;
            break;
        }*/
        /*usleep(5000);
    }*/

        /*if(mpViewer)
            pangolin::BindToContext("ORB-SLAM2: Map Viewer");*/

#ifdef REGISTER_TIMES
        mpTracker->PrintTimeStats();
#endif

    }

    bool System::isShutDown()
    {
        unique_lock<mutex> lock(mMutexReset);
        return mbShutDown;
    }

    int System::GetTrackingState()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackingState;
    }

    std::vector<MapPoint *> System::GetTrackedMapPoints()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedMapPoints;
    }

    std::vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedKeyPointsUn;
    }

    bool System::isLost()
    {

        if ((mpTracker->mState==Tracking::LOST)) //||(mpTracker->mState==Tracking::RECENTLY_LOST))
            return true;
        else
            return false;
    }

    bool System::isFinished()
    {
        return true;
    }
}
