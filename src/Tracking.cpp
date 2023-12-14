/**
  ******************************************************************************
  * @file           : Tracking.cpp
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/13
  ******************************************************************************
 **/

#include "Tracking.h"
#include "Optimizer.h"
#include "MLPnPsolver.h"


namespace LoFTR_SLAM
{

    Tracking::Tracking(System *pSys, FrameDrawer *pFrameDrawer, MapDrawer* pMapDrawer, Atlas *pAtlas, LoFTRmatcher *pMatcher,
                       const string &strSettingPath, const string &strMaskPath, const int sensor, Settings *settings):
    mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false), mpLoFTRmatcher(pMatcher),
    mbOnlyTracking(true), mbMapUpdated(false), mbVO(false),
    mbReadyToInitializate(false), mpSystem(pSys), mpViewer(NULL), bStepByStep(false),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnLastRelocFrameId(0),
    mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpLastKeyFrame(static_cast<KeyFrame*>(NULL))
    {
        // Load mask imaged
        mImMask = cv::imread(strMaskPath, cv::IMREAD_GRAYSCALE);

        // Load camera parameters from settings file
        if(settings)
        {
            newParameterLoader(settings);
        }
        else
        {
            cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

            bool b_parse_cam = ParseCamParamFile(fSettings);
            if(!b_parse_cam)
            {
                std::cout << "*Error with the camera parameters in the config file*" << std::endl;
            }


            if(!b_parse_cam)
            {
                std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
                try
                {
                    throw -1;
                }
                catch(exception &e)
                {

                }
            }
        }

        initID = 0; lastID = 0;

        vector<GeometricCamera*> vpCams = mpAtlas->GetAllCameras();
        std::cout << "There are " << vpCams.size() << " cameras in the atlas" << std::endl;
        for(GeometricCamera* pCam : vpCams)
        {
            std::cout << "Camera " << pCam->GetId();
            if(pCam->GetType() == GeometricCamera::CAM_PINHOLE)
            {
                std::cout << " is pinhole" << std::endl;
            }
            else if(pCam->GetType() == GeometricCamera::CAM_FISHEYE)
            {
                std::cout << " is fisheye" << std::endl;
            }
            else
            {
                std::cout << " is unknown" << std::endl;
            }
        }

#ifdef REGISTER_TIMES
        vdRectStereo_ms.clear();
    vdResizeImage_ms.clear();
    vdORBExtract_ms.clear();
    vdStereoMatch_ms.clear();
    vdIMUInteg_ms.clear();
    vdPosePred_ms.clear();
    vdLMTrack_ms.clear();
    vdNewKF_ms.clear();
    vdTrackTotal_ms.clear();
#endif
    }

    void Tracking::newParameterLoader(Settings *settings)
    {
        mpCamera = settings->camera();
        mpCamera = mpAtlas->AddCamera(mpCamera);

        if(settings->needToUndistort()){
            mDistCoef = settings->cameraDistortionCoef();
        }
        else{
            mDistCoef = cv::Mat::zeros(4,1,CV_32F);
        }

        //TODO: missing image scaling and rectification
        mImageScale = 1.0f;

        mK = cv::Mat::eye(3,3,CV_32F);
        mK.at<float>(0,0) = mpCamera->getParameter(0);
        mK.at<float>(1,1) = mpCamera->getParameter(1);
        mK.at<float>(0,2) = mpCamera->getParameter(2);
        mK.at<float>(1,2) = mpCamera->getParameter(3);

        mK_.setIdentity();
        mK_(0,0) = mpCamera->getParameter(0);
        mK_(1,1) = mpCamera->getParameter(1);
        mK_(0,2) = mpCamera->getParameter(2);
        mK_(1,2) = mpCamera->getParameter(3);

        mMinFrames = 0;
        mMaxFrames = settings->fps();
        mbRGB = settings->rgb();
    }

    bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings)
    {
        mDistCoef = cv::Mat::zeros(4,1,CV_32F);
        cout << endl << "Camera Parameters: " << endl;
        bool b_miss_params = false;

        string sCameraName = fSettings["Camera.type"];
        if(sCameraName == "PinHole")
        {
            float fx, fy, cx, cy;
            mImageScale = 1.f;

            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera.fx"];
            if(!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.fy"];
            if(!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cx"];
            if(!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cy"];
            if(!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera.k1"];
            if(!node.empty() && node.isReal())
            {
                mDistCoef.at<float>(0) = node.real();
            }
            else
            {
                std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k2"];
            if(!node.empty() && node.isReal())
            {
                mDistCoef.at<float>(1) = node.real();
            }
            else
            {
                std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.p1"];
            if(!node.empty() && node.isReal())
            {
                mDistCoef.at<float>(2) = node.real();
            }
            else
            {
                std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.p2"];
            if(!node.empty() && node.isReal())
            {
                mDistCoef.at<float>(3) = node.real();
            }
            else
            {
                std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k3"];
            if(!node.empty() && node.isReal())
            {
                mDistCoef.resize(5);
                mDistCoef.at<float>(4) = node.real();
            }

            node = fSettings["Camera.imageScale"];
            if(!node.empty() && node.isReal())
            {
                mImageScale = node.real();
            }

            if(b_miss_params)
            {
                return false;
            }

            if(mImageScale != 1.f)
            {
                // K matrix parameters must be scaled.
                fx = fx * mImageScale;
                fy = fy * mImageScale;
                cx = cx * mImageScale;
                cy = cy * mImageScale;
            }

            vector<float> vCamCalib{fx,fy,cx,cy};

            mpCamera = new Pinhole(vCamCalib);

            mpCamera = mpAtlas->AddCamera(mpCamera);

            std::cout << "- Camera: Pinhole" << std::endl;
            std::cout << "- Image scale: " << mImageScale << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
            std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;


            std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
            std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

            if(mDistCoef.rows==5)
                std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

            mK = cv::Mat::eye(3,3,CV_32F);
            mK.at<float>(0,0) = fx;
            mK.at<float>(1,1) = fy;
            mK.at<float>(0,2) = cx;
            mK.at<float>(1,2) = cy;

            mK_.setIdentity();
            mK_(0,0) = fx;
            mK_(1,1) = fy;
            mK_(0,2) = cx;
            mK_(1,2) = cy;
        }
        else if(sCameraName == "KannalaBrandt8")
        {
            float fx, fy, cx, cy;
            float k1, k2, k3, k4;
            mImageScale = 1.f;

            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera.fx"];
            if(!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera.fy"];
            if(!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cx"];
            if(!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cy"];
            if(!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera.k1"];
            if(!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera.k2"];
            if(!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k3"];
            if(!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k4"];
            if(!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.imageScale"];
            if(!node.empty() && node.isReal())
            {
                mImageScale = node.real();
            }

            if(!b_miss_params)
            {
                if(mImageScale != 1.f)
                {
                    // K matrix parameters must be scaled.
                    fx = fx * mImageScale;
                    fy = fy * mImageScale;
                    cx = cx * mImageScale;
                    cy = cy * mImageScale;
                }

                vector<float> vCamCalib{fx,fy,cx,cy,k1,k2,k3,k4};
                mpCamera = new KannalaBrandt8(vCamCalib);
                mpCamera = mpAtlas->AddCamera(mpCamera);
                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- Image scale: " << mImageScale << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                mK = cv::Mat::eye(3,3,CV_32F);
                mK.at<float>(0,0) = fx;
                mK.at<float>(1,1) = fy;
                mK.at<float>(0,2) = cx;
                mK.at<float>(1,2) = cy;

                mK_.setIdentity();
                mK_(0,0) = fx;
                mK_(1,1) = fy;
                mK_(0,2) = cx;
                mK_(1,2) = cy;
            }

            if(b_miss_params)
            {
                return false;
            }

        }
        else
        {
            std::cerr << "*Not Supported Camera Sensor*" << std::endl;
            std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
        }

        float fps = fSettings["Camera.fps"];
        if(fps==0)
            fps=30;

        // Max/Min Frames to insert keyframes and to check relocalisation
        mMinFrames = 0;
        mMaxFrames = fps;

        cout << "- fps: " << fps << endl;


        int nRGB = fSettings["Camera.RGB"];
        mbRGB = nRGB;

        if(mbRGB)
            cout << "- color order: RGB (ignored if grayscale)" << endl;
        else
            cout << "- color order: BGR (ignored if grayscale)" << endl;

        if(b_miss_params)
        {
            return false;
        }

        return true;
    }

    void Tracking::SetViewer(Viewer *pViewer) {
        mpViewer=pViewer;
    }

    float Tracking::GetImageScale() {
        return mImageScale;
    }

    void Tracking::InformOnlyTracking(const bool &flag) {
        mbOnlyTracking = true;
    }

    void Tracking::Reset(bool bLocMap)
    {
        Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

        if(mpViewer)
        {
            mpViewer->RequestStop();
            while(!mpViewer->isStopped())
                usleep(3000);
        }

        // Clear Map (this erase MapPoints and KeyFrames)
        mpAtlas->clearAtlas();
        mpAtlas->CreateNewMap();
        mnInitialFrameId = 0;

        KeyFrame::nNextId = 0;
        Frame::nNextId = 0;
        mState = NO_IMAGES_YET;

        mbReadyToInitializate = false;
        mbSetInit=false;

        mlRelativeFramePoses.clear();
        mlpReferences.clear();
        mlFrameTimes.clear();
        mlbLost.clear();
        mCurrentFrame = Frame();
        mnLastRelocFrameId = 0;
        mLastFrame = Frame();
        mpReferenceKF = static_cast<KeyFrame*>(NULL);
        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
        mvIniMatches.clear();

        if(mpViewer)
            mpViewer->Release();

        Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
    }

    void Tracking::ResetActiveMap(bool bLocMap)
    {
        Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
        if(mpViewer)
        {
            mpViewer->RequestStop();
            while(!mpViewer->isStopped())
                usleep(3000);
        }

        Map* pMap = mpAtlas->GetCurrentMap();

        // Clear Map (this erase MapPoints and KeyFrames)
        mpAtlas->clearMap();

        //KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
        //Frame::nNextId = mnLastInitFrameId;
        mnLastInitFrameId = Frame::nNextId;
        //mnLastRelocFrameId = mnLastInitFrameId;
        mState = NO_IMAGES_YET; //NOT_INITIALIZED;

        mbReadyToInitializate = false;

        list<bool> lbLost;
        // lbLost.reserve(mlbLost.size());
        unsigned int index = mnFirstFrameId;
        cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
        for(Map* pMap : mpAtlas->GetAllMaps())
        {
            if(pMap->GetAllKeyFrames().size() > 0)
            {
                if(index > pMap->GetLowerKFID())
                    index = pMap->GetLowerKFID();
            }
        }

        //cout << "First Frame id: " << index << endl;
        int num_lost = 0;
        cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

        for(list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++)
        {
            if(index < mnInitialFrameId)
                lbLost.push_back(*ilbL);
            else
            {
                lbLost.push_back(true);
                num_lost += 1;
            }

            index++;
        }
        cout << num_lost << " Frames set to lost" << endl;

        mlbLost = lbLost;

        mnInitialFrameId = mCurrentFrame.mnId;
        mnLastRelocFrameId = mCurrentFrame.mnId;

        mCurrentFrame = Frame();
        mLastFrame = Frame();
        mpReferenceKF = static_cast<KeyFrame*>(NULL);
        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
        mvIniMatches.clear();

        if(mpViewer)
            mpViewer->Release();

        Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
    }

    Sophus::SE3f Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
    {
        // 转化成灰度图像
        mImGray = im;
        if(mImGray.channels()==3)
        {
            if(mbRGB)
                cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
            else
                cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
        }
        else if(mImGray.channels()==4)
        {
            if(mbRGB)
                cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
            else
                cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
        }

        // 创建当前帧
        if (mSensor == System::MONOCULAR || mSensor==System::MONOCULAR_EMT)
        {
            mCurrentFrame = Frame(mImGray, timestamp, mpCamera, mDistCoef, &mLastFrame);
        }

        if (mState==NO_IMAGES_YET)
            t0=timestamp;

#ifdef REGISTER_TIMES
        vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif
        lastID = mCurrentFrame.mnId;
        Track();

        return mCurrentFrame.GetPose();
    }

    void Tracking::Track()
    {
        // track包含两部分：估计运动、跟踪局部地图

        // 设置部分，是否逐步进行跟踪
        if (bStepByStep)
        {
            std::cout << "Tracking: Waiting to the next step" << std::endl;
            while(!mbStep && bStepByStep)
                usleep(500);
            mbStep = false;
        }

        // 获得当前Map
        Map* pCurrentMap = mpAtlas->GetCurrentMap();

        if(!pCurrentMap)
        {
            cout << "ERROR: There is not an active map in the atlas" << endl;
        }

        // 如果当前帧不是第一帧时，出现时间戳错乱或跳帧情况的话
        if(mState!=NO_IMAGES_YET)
        {
            // 时间戳错乱，则创建新的Map
            if(mLastFrame.mTimeStamp>mCurrentFrame.mTimeStamp)
            {
                cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
                CreateMapInAtlas();
                return;
            }
                // 时间戳没有错乱，但是跳帧，则Reset当前的Map
            else if(mCurrentFrame.mTimeStamp>mLastFrame.mTimeStamp+1.0)
            {
                cout << "Timestamp jump detected. State set to LOST." << endl;
                mpSystem->ResetActiveMap();
            }
        }

        // 如果图像复位过、或者第一次运行，则为NO_IMAGE_YET
        if(mState==NO_IMAGES_YET)
        {
            mState = NOT_INITIALIZED;
        }

        // mLastProcessedState 存储了Tracking最新的状态，用于FrameDrawer中的绘制
        mLastProcessedState=mState;

        mbCreatedMap = false;

        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

        mbMapUpdated = false;

        // 判断当前地图是否跟新
        int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
        int nMapChangeIndex = pCurrentMap->GetLastMapChange();
        if(nCurMapChangeIndex>nMapChangeIndex)
        {
            pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
            mbMapUpdated = true;
        }

        // Step 1: 初始化
        // 判断是否进行初始化，如果没有进行初始化，则初始化；如果已经初始化完成，则跟踪当前帧
        if(mState==NOT_INITIALIZED)
        {
//            // 进行单目相机初始化
//            if (!mbReadyToInitializate)
//            {
//                mImInit = mImGray.clone();
//            }
//            else
//            {
//                mImCurr = mImGray.clone();
//            }
            MonocularInitialization();


            // 判断初始化是否成功
            // If rightly initialized, mState=OK
            if(mState!=OK)
            {
                mLastFrame = Frame(mCurrentFrame);
                return;
            }

            // 判断是否成功创建地图，成功则将第一帧设置为当前帧
            if(mpAtlas->GetAllMaps().size() == 1)
            {
                mnFirstFrameId = mCurrentFrame.mnId;
            }
        }
        else
        {
            // System is initialized. Track Frame.
            // bOK为临时变量，用于表示每个函数是否执行成功
            bool bOK;

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartPosePred = std::chrono::steady_clock::now();
#endif

            // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
            // 判断是否需要构建局部地图，初始化相机位姿（成功的话bOK == true）
            // tracking 类构造时默认为false。在viewer中有个开关ActivateLocalizationMode，可以控制是否开启mbOnlyTracking
            if(!mbOnlyTracking)
            {
                // State OK
                // Local Mapping is activated. This is the normal behaviour, unless
                // you explicitly activate the "only tracking" mode.

                // Step 2. 进入正常的SLAM模式，有地图更新
                // 正常初始化成功
                if(mState==OK)
                {

                    // Local Mapping might have changed some MapPoints tracked in last frame
                    // 检查并更新上一帧被替换的地图点
                    CheckReplacedInLastFrame();

                    // 利用 trackreferenceKeyframe 的方法进行跟踪
                    // 用最近的关键帧来跟踪当前帧特征点的匹配点
                    Verbose::PrintMess("TRACK: Track with respect to the reference KF ", Verbose::VERBOSITY_DEBUG);

                    bOK = TrackReferenceKeyFrame();

                    // 如果跟踪不成功,则将状态设置为暂时丢失或者丢失
                    if (!bOK)
                    {
                        if(pCurrentMap->KeyFramesInMap()>10)
                        {
                            cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
                            mState = RECENTLY_LOST;
                            mTimeStampLost = mCurrentFrame.mTimeStamp;
                        }
                        else
                        {
                            mState = LOST;
                        }
                    }
                }
                else
                {
                    // 如果当前状态为LOST/RECENTLY_LOST，进行重定位，重定位失败则丢失重新创建地图
                    if (mState == RECENTLY_LOST)
                    {
                        cout << "Lost for a short time" << endl;
                        Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);

                        bOK = true;
                        bOK = Relocalization();
                        if(mCurrentFrame.mTimeStamp-mTimeStampLost>3.0f && !bOK)
                        {
                            mState = LOST;
                            cout << "Track lost ..." << endl;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                    else if (mState == LOST)
                    {
                        cout << "A new map is started..." << endl;
                        Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL);

                        if (pCurrentMap->KeyFramesInMap()<10)
                        {
                            mpSystem->ResetActiveMap();
                            cout << "Resetting current map..." << endl;
                            Verbose::PrintMess("Resetting current map...", Verbose::VERBOSITY_NORMAL);
                        }
                        else
                            CreateMapInAtlas();

                        if(mpLastKeyFrame)
                            mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

                        cout << "done" << endl;
                        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

                        return;
                    }
                }
            }
            else
            {
                // Localization Mode: Local Mapping is deactivated
                if(mState==LOST)
                {
                    bOK = Relocalization();
                }
                else
                {
                    // mbVO 是mbOnlyTracking 为 true 才会有的一个变量
                    // mbVO 为false表示帧匹配了很多的MapPoints，跟踪正常
                    // mbVO 为true表示帧匹配的MapPoints很少，要寄
                    if(!mbVO)
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                    else
                    {
                        // In last frame we tracked mainly "visual odometry" points.
                        // mbVO为 true，说明跟踪的特征点很少，就要进行重定位了
                        bool bOKReloc = false;

                        bOKReloc = Relocalization();

                        if(bOKReloc)
                        {
                            mbVO = false;
                        }

                        bOK = bOKReloc;
                    }
                }
            }

            // 将最新参考帧设置当前帧的参考关键帧
            if(!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndPosePred = std::chrono::steady_clock::now();

        double timePosePred = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPosePred - time_StartPosePred).count();
        vdPosePred_ms.push_back(timePosePred);
#endif


#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartLMTrack = std::chrono::steady_clock::now();
#endif
            // Step 3: 跟踪得到当前帧的初始位姿后，对local map 进行跟踪，获得更多的匹配，并优化当前位姿
            // 前面只是跟踪一帧得到初始位姿，这里搜索局部关键帧、局部地图点，和当前帧进行投影匹配，得到更多匹配的MapPoints后进行优化
            // If we have an initial estimation of the camera pose and matching. Track the local map.
            // 如果局部地图跟踪成功，则更新速度，按需添加关键帧
            if(!mbOnlyTracking)
            {
                if(bOK)
                {
                    bOK = TrackLocalMap();
                }
                if(!bOK)
                    cout << "Fail to track local map!" << endl;
            }
            else
            {
                // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
                // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
                // the camera we will use the local map again.
                if(bOK && !mbVO)
                    bOK = TrackLocalMap();
            }

            // 根据跟踪和跟踪局部地图结果更新mState
            if(bOK)
                mState = OK;
            else if (mState == OK)
            {
                mState=RECENTLY_LOST; // visual to lost

                /*if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
                {*/
                mTimeStampLost = mCurrentFrame.mTimeStamp;
                //}
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndLMTrack = std::chrono::steady_clock::now();

        double timeLMTrack = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLMTrack - time_StartLMTrack).count();
        vdLMTrack_ms.push_back(timeLMTrack);
#endif

            // Update drawer
            mpFrameDrawer->Update(this);
            if(mCurrentFrame.isSet())
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

            // 清除观测不到的地图点，关键帧的创建，删除外点
            if(bOK || mState==RECENTLY_LOST)
            {
                // Clean VO matches
                // Step 6：清除观测不到的地图点
                for(int i=0; i<mCurrentFrame.N; i++)
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                    if(pMP)
                        if(pMP->Observations()<1)
                        {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                        }
                }

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_StartNewKF = std::chrono::steady_clock::now();
#endif
                bool bNeedKF = NeedNewKeyFrame();

                // Check if we need to insert a new keyframe
                // if(bNeedKF && bOK)
                if(bNeedKF && bOK)
                    CreateNewKeyFrame();

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndNewKF = std::chrono::steady_clock::now();

            double timeNewKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndNewKF - time_StartNewKF).count();
            vdNewKF_ms.push_back(timeNewKF);
#endif

                // Step 9 删除那些在BA中被设置为外点的地图点
                // We allow points with high innovation (considererd outliers by the Huber Function)
                // pass to the new keyframe, so that bundle adjustment will finally decide
                // if they are outliers or not. We don't want next frame to estimate its position
                // with those points so we discard them in the frame. Only has effect if lastframe is tracked
                for(int i=0; i<mCurrentFrame.N;i++)
                {
                    if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                }
            }

            // Step 4. 如果初始化后不久就跟踪失败，而且 relocalization 也没能搞定，只能重新Reset
            // Reset if the camera get lost soon after initialization
            if(mState==LOST)
            {
                if(pCurrentMap->KeyFramesInMap()<=5)
                {
                    mpSystem->ResetActiveMap();
                    return;
                }

                CreateMapInAtlas();

                return;
            }

            // 确保已经设置了参考关键帧
            if(!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

            // 将当前帧设置成上一帧
            mLastFrame = Frame(mCurrentFrame);
        }


        if(mState==OK || mState==RECENTLY_LOST)
        {
            // Step 11. 记录位姿信息，用于最后保存所有的轨迹
            // Store frame pose information to retrieve the complete camera trajectory afterwards.
            if(mCurrentFrame.isSet())
            {
                Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() * mCurrentFrame.mpReferenceKF->GetPoseInverse();
                //  保存各种状态
                mlRelativeFramePoses.push_back(Tcr_);
                mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
                mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
                mlbLost.push_back(mState==LOST);
            }
            else
            {
                // This can happen if tracking is lost
                // 如果跟踪失败，则相对位置使用上一次值
                mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
                mlpReferences.push_back(mlpReferences.back());
                mlFrameTimes.push_back(mlFrameTimes.back());
                mlbLost.push_back(mState==LOST);
            }

        }

#ifdef REGISTER_LOOP
        if (Stop()) {

        // Safe area to stop
        while(isStopped())
        {
            usleep(3000);
        }
    }
#endif
    }

    void Tracking::CreateMapInAtlas()
    {
        mnLastInitFrameId = mCurrentFrame.mnId;
        mpAtlas->CreateNewMap();

        mbSetInit=false;

        mnInitialFrameId = mCurrentFrame.mnId+1;
        mState = NO_IMAGES_YET;

        //mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the current id, because it is the new starting point for new map
        Verbose::PrintMess("First frame id in map: " + to_string(mnLastInitFrameId+1), Verbose::VERBOSITY_NORMAL);
        mbVO = false; // Init value for know if there are enough MapPoints in the last KF
        if(mSensor == System::MONOCULAR)
        {
            mbReadyToInitializate = false;
        }

        if(mpLastKeyFrame)
            mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

        if(mpReferenceKF)
            mpReferenceKF = static_cast<KeyFrame*>(NULL);

        mLastFrame = Frame();
        mCurrentFrame = Frame();
        mvIniMatches.clear();

        mbCreatedMap = true;
    }

    void Tracking::MonocularInitialization()
    {
        if(!mbReadyToInitializate)      // 判断是否符合初始化条件，如果不符合说明是第一帧，如果符合则是第二帧
        {
            // Set Reference Frame
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

            mbReadyToInitializate = true;

            return;
        }
        else    // 第二帧
        {
            // Find correspondences 找匹配点， 因为没有提取特征点，所以直接在这一步将提取的特征点放到两帧图像中
            int nmatches = mpLoFTRmatcher->SearchForInitialization(mInitialFrame, mCurrentFrame, mvIniMatches);

            // Check if there are enough correspondences，否则重新来过
            if(nmatches<100)
            {
                mbReadyToInitializate = false;
                return;
            }

            Sophus::SE3f Tcw;
            vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
            bool isReconstruct = false;
            isReconstruct = mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvIniMatches,Tcw,mvIniP3D,vbTriangulated);

            if(isReconstruct)
            {   //如果位姿求解成功，删除没有成功三角化的匹配点
                for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
                {
                    if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                    {
                        mvIniMatches[i]=-1;
                        nmatches--;
                    }
                }

                // Set Frame Poses 设置两帧的位姿
                mInitialFrame.SetPose(Sophus::SE3f());
                mCurrentFrame.SetPose(Tcw);
                //根据三角化的结果，创建初始地图点，并做一些必要的计算
                // TODO Adjacent reconstruction
                CreateInitialMapMonocular();
            }
        }
    }

    void Tracking::CreateInitialMapMonocular()
    {
        // Create KeyFrames 认为单目初始化时候的参考帧和当前帧都是关键帧
        KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpAtlas->GetCurrentMap());
        KeyFrame* pKFcur = new KeyFrame(mInitialFrame, mpAtlas->GetCurrentMap());


        // Insert KFs in the map
        // Step 1. 将关键帧插入到地图
        mpAtlas->AddKeyFrame(pKFini);
        mpAtlas->AddKeyFrame(pKFcur);

        // Step 2. 初始化3d点来生成地图点MapPoints
        int ncorres = 0;
        for(size_t i=0; i<mvIniMatches.size();i++)
        {
            if(mvIniMatches[i]<0)
                continue;

            // count corresponding points
            ncorres++;

            //Create MapPoint.
            Eigen::Vector3f worldPos;
            // 3维点坐标
            worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;

            // Step 2.1 构造3D点
            MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpAtlas->GetCurrentMap());

            // Step 2.2 为MapPoint 添加属性: 观测到该点的关键帧； 该点的平均观测方向和深度范围
            pKFini->AddMapPoint(pMP,i);
            pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

            //  添加帧之间的可见信息
            // 表示该MapPoint可以被哪个KeyFrame的哪个特征点观测到
            pMP->AddObservation(pKFini,i);
            pMP->AddObservation(pKFcur,mvIniMatches[i]);

            // 计算平均观测方向以及观测距离的范围
            pMP->UpdateNormal();

            //Fill Current Frame structure
            mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

            //Add to Map
            mpAtlas->AddMapPoint(pMP);
        }

        // Update Connections 更新共视图的边和权重
        // Step 2.3 更新关键帧之间的连接
        // 在3D点和关键帧之间建立边，每个边有一个权重，边的权重是该关键帧与当前帧公共3D点的个数
        pKFini->UpdateConnections();
        pKFcur->UpdateConnections();

        // Bundle Adjustment
        Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);
        Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(),20);

        // 对整个场景的尺度归一化，常用的方法是令第0帧和第1帧之间的平移距离为单位1。
        // 这里采用令第0帧场景深度中位值为单位1的方法，来控制场景的规模。
        // 首先调用ComputeSceneMedianDepth()计算第0帧的深度中位值，确定尺度。
        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        float invMedianDepth;

        invMedianDepth = 1.0f/medianDepth;

        // 两个条件，一个是平均深度要大于0，另一个是在当前帧中被观测的地图点数大于100
        if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100) // TODO Check, originally 100 tracks
        {
            Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_QUIET);
            mpSystem->ResetActiveMap();
            return;
        }

        // 用得到的尺度归一化关键帧位姿和地图点坐标。
        // Scale initial baseline
        Sophus::SE3f Tc2w = pKFcur->GetPose();
        Tc2w.translation() *= invMedianDepth;
        pKFcur->SetPose(Tc2w);

        // Scale points
        // 把3D点的尺度也归一化到1
        vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
        for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
        {
            if(vpAllMapPoints[iMP])
            {
                MapPoint* pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
                pMP->UpdateNormal();
            }
        }

        mCurrentFrame.SetPose(pKFcur->GetPose());
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFcur;
        //mnLastRelocFrameId = mInitialFrame.mnId;

        mvpLocalKeyFrames.push_back(pKFcur);
        mvpLocalKeyFrames.push_back(pKFini);
        // 初始化之后，得到的地图点都是局部地图点
        mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFcur;
        mCurrentFrame.mpReferenceKF = pKFcur;

        mLastFrame = Frame(mCurrentFrame);

        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mState=OK;

        initID = pKFcur->mnId;
    }

    void Tracking::CheckReplacedInLastFrame()
    {
        for(int i =0; i<mLastFrame.N; i++)
        {
            MapPoint* pMP = mLastFrame.mvpMapPoints[i];
            //如果这个地图点存在
            if(pMP)
            {
                // 获取其是否被替换,以及替换后的点
                // 这也是程序不直接删除这个地图点删除的原因
                MapPoint* pRep = pMP->GetReplaced();
                if(pRep)
                {
                    //然后替换一下
                    mLastFrame.mvpMapPoints[i] = pRep;
                }
            }
        }

    }

    bool Tracking::TrackReferenceKeyFrame()
    {
        vector<MapPoint*> vpMapPointMatches;
        // 首先利用LoFTRmatcher 寻找当前帧和上一帧之间的匹配对,获得对应的特征点
        int nmatches = mpLoFTRmatcher->SearchForTrack(mpReferenceKF, mCurrentFrame, vpMapPointMatches);


        // 进行当前帧和参考帧之间的特征匹配，对应参考关键帧的地图点的指针将被保存在向量vpMapPointMatches中
        if(nmatches<15)
        {
            cout << "TRACK_REF_KF: Less than 15 matches!!\n";
            return false;
        }

        // 将当前帧的地图点更新为对应到参考关键帧中的地图点
        mCurrentFrame.mvpMapPoints = vpMapPointMatches;
        // 将当前帧的位姿设置成上一帧的位姿
        mCurrentFrame.SetPose(mLastFrame.GetPose());

        // 对位姿进行优化
        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        // 抛弃外点
        int nmatchesMap = 0;
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;

                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        }

        // 最终的匹配点数如果大于10就说明跟踪成功
        return nmatchesMap>=10;
    }

    void Tracking::UpdateLastFrame()
    {
        // Update pose according to reference keyframe
        // Step 1：计算上一帧在世界坐标系下的位姿
        // 上一普通帧的参考关键帧，注意这里用的是参考关键帧（位姿准）而不是上上一帧的普通帧
        KeyFrame* pRef = mLastFrame.mpReferenceKF;
        Sophus::SE3f Tlr = mlRelativeFramePoses.back();
        mLastFrame.SetPose(Tlr * pRef->GetPose());

        if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || mSensor==System::MONOCULAR_EMT || !mbOnlyTracking)
            return;
    }

    bool Tracking::Relocalization()
    {
        Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);

        // Step 1 获得参考关键帧
        int nMaxKFs = 10;
        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        vector<KeyFrame*> vpAllKeyFrames = mpAtlas->GetCurrentMap()->GetAllKeyFrames();
        vector<KeyFrame*> vpCandidateKFs;
        if (vpAllKeyFrames.size() < 10)
        {
            vpCandidateKFs = vpAllKeyFrames;
        }
        else
        {
            vpCandidateKFs.insert(vpCandidateKFs.begin(), vpAllKeyFrames.end() - nMaxKFs, vpAllKeyFrames.end());
        }

        if(vpCandidateKFs.empty()) {
            Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
            return false;
        }

        const int nKFs = vpCandidateKFs.size();

        // We perform first an LoFTR matching with each candidate
        // If enough matches are found we setup a PnP solver
        vector<MLPnPsolver*> vpMLPnPsolvers;
        vpMLPnPsolvers.resize(nKFs);

        vector<vector<MapPoint*>> vvpMapPointMatches;  // 用于存储每个候选关键帧的mappoint
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates=0;

        for (int i=0; i<nKFs; i++)
        {
            KeyFrame* pKF = vpCandidateKFs[i];
            if (pKF->isBad())
            {
                vbDiscarded[i] = true;
            }
            else
            {
                int nmatches = mpLoFTRmatcher->SearchForTrack(pKF, mCurrentFrame, vvpMapPointMatches[i]);
                if (nmatches < 15) {
                    vbDiscarded[i] = true;
                    continue;
                }
                else
                {
                    MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99, 10, 300, 6, 0.5, 5.991);
                    vpMLPnPsolvers[i] = pSolver;
                    nCandidates++;
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9,true);

        while(nCandidates>0 && !bMatch)
        {
            for(int i=0; i<nKFs; i++)
            {
                if(vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                MLPnPsolver* pSolver = vpMLPnPsolvers[i];
                Eigen::Matrix4f eigTcw;
                bool bTcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers, eigTcw);

                // If Ransac reachs max. iterations discard keyframe
                if(bNoMore)
                {
                    vbDiscarded[i]=true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if(bTcw)
                {
                    Sophus::SE3f Tcw(eigTcw);
                    mCurrentFrame.SetPose(Tcw);
                    // Tcw.copyTo(mCurrentFrame.mTcw);

                    set<MapPoint*> sFound;

                    const int np = vbInliers.size();

                    for(int j=0; j<np; j++)
                    {
                        if(vbInliers[j])
                        {
                            mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        }
                        else
                            mCurrentFrame.mvpMapPoints[j]=NULL;
                    }

                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                    if(nGood<10)
                        continue;

                    for(int io =0; io<mCurrentFrame.N; io++)
                        if(mCurrentFrame.mvbOutlier[io])
                            mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);


                    // If the pose is supported by enough inliers stop ransacs and continue
                    if(nGood>=50)
                    {
                        bMatch = true;
                        break;
                    }
                }
            }
        }

        if(!bMatch)
        {
            return false;
        }
        else
        {
            mnLastRelocFrameId = mCurrentFrame.mnId;
            cout << "Relocalized!!" << endl;
            return true;
        }
    }

    void Tracking::UpdateLocalMap()
    {
        // This is for visualization
        // 设置参考地图点用于绘图显示局部地图点（红色）
        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        // Update
        // 用共视图来更新局部关键帧和局部地图点
        UpdateLocalKeyFrames();
        UpdateLocalPoints();
    }

    void Tracking::UpdateLocalPoints()
    {
        // Step 1：清空局部MapPoints
        mvpLocalMapPoints.clear();

        int count_pts = 0;

        // Step 2：遍历局部关键帧 mvpLocalKeyFrames
        for(vector<KeyFrame*>::const_reverse_iterator itKF=mvpLocalKeyFrames.rbegin(), itEndKF=mvpLocalKeyFrames.rend(); itKF!=itEndKF; ++itKF)
        {
            KeyFrame* pKF = *itKF;
            const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

            for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
            {

                MapPoint* pMP = *itMP;
                if(!pMP)
                    continue;
                if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                    continue;
                if(!pMP->isBad())
                {
                    count_pts++;
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                }
            }
        }
    }

    void Tracking::UpdateLocalKeyFrames()
    {
        // Each map point vote for the keyframes in which it has been observed
        map<KeyFrame*,int> keyframeCounter;
        if((mCurrentFrame.mnId<mnLastRelocFrameId+2))
        {
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                {
                    if(!pMP->isBad())
                    {
                        const map<KeyFrame*,int > observations = pMP->GetObservations();
                        for(map<KeyFrame*,int>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                            keyframeCounter[it->first]++;
                    }
                    else
                    {
                        mCurrentFrame.mvpMapPoints[i]=NULL;
                    }
                }
            }
        }
        else
        {
            for(int i=0; i<mLastFrame.N; i++)
            {
                // Using lastframe since current frame has not matches yet
                if(mLastFrame.mvpMapPoints[i])
                {
                    MapPoint* pMP = mLastFrame.mvpMapPoints[i];
                    if(!pMP)
                        continue;
                    if(!pMP->isBad())
                    {
                        const map<KeyFrame*,int> observations = pMP->GetObservations();
                        for(map<KeyFrame*,int >::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                            keyframeCounter[it->first]++;
                    }
                    else
                    {
                        // MODIFICATION
                        mLastFrame.mvpMapPoints[i]=NULL;
                    }
                }
            }
        }


        int max=0;
        KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

        mvpLocalKeyFrames.clear();
        mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
        {
            KeyFrame* pKF = it->first;

            if(pKF->isBad())
                continue;

            if(it->second>max)
            {
                max=it->second;
                pKFmax=pKF;
            }

            mvpLocalKeyFrames.push_back(pKF);
            pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        }

        // Include also some not-already-included keyframes that are neighbors to already-included keyframes
        for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
        {
            // Limit the number of keyframes
            if(mvpLocalKeyFrames.size()>80) // 80
                break;

            KeyFrame* pKF = *itKF;

            const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);


            for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
            {
                KeyFrame* pNeighKF = *itNeighKF;
                if(!pNeighKF->isBad())
                {
                    if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pNeighKF);
                        pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            const set<KeyFrame*> spChilds = pKF->GetChilds();
            for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
            {
                KeyFrame* pChildKF = *sit;
                if(!pChildKF->isBad())
                {
                    if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pChildKF);
                        pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            KeyFrame* pParent = pKF->GetParent();
            if(pParent)
            {
                if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pParent);
                    pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        if(pKFmax)
        {
            mpReferenceKF = pKFmax;
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
    }

    bool Tracking::TrackLocalMap()
    {
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.
        mTrackedFr++;

        UpdateLocalMap();

        // TOO check outliers before PO
        int aux1 = 0, aux2=0;
        for(int i=0; i<mCurrentFrame.N; i++)
            if( mCurrentFrame.mvpMapPoints[i])
            {
                aux1++;
                if(mCurrentFrame.mvbOutlier[i])
                    aux2++;
            }

        int inliers;

        Optimizer::PoseOptimization(&mCurrentFrame);

        aux1 = 0, aux2 = 0;
        for(int i=0; i<mCurrentFrame.N; i++)
            if( mCurrentFrame.mvpMapPoints[i])
            {
                aux1++;
                if(mCurrentFrame.mvbOutlier[i])
                    aux2++;
            }

        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(!mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if(!mbOnlyTracking)
                    {
                        if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                            mnMatchesInliers++;
                    }
                    else
                        mnMatchesInliers++;
                }
            }
        }

        // Decide if the tracking was succesful
        cout << "The number of matches inliers is " + to_string(mnMatchesInliers) << endl;
        if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
            return false;

        if((mnMatchesInliers>10)&&(mState==RECENTLY_LOST))
            return true;

        if(mnMatchesInliers<30) // 原来是30
            return false;
        else
            return true;
    }

    bool Tracking::NeedNewKeyFrame()
    {
        // 纯VO模式不加入关键帧
        if(mbOnlyTracking)
            return false;

        // 获得当前关键帧的数目
        const int nKFs = mpAtlas->KeyFramesInMap();

        // Do not insert keyframes if not enough frames have passed from last relocalisation
        // 如果距离上一次重定位比较近，并且关键帧数目超出最大限制，不插入关键帧
        if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        {
            return false;
        }

        // Tracked MapPoints in the reference keyframe
        // 得到参考关键帧跟踪到的地图点数
        int nMinObs = 3;
        if(nKFs<=2)
            nMinObs=2;
        int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

        // Check how many "close" points are being tracked and how many could be potentially created.
        int nNonTrackedClose = 0;
        int nTrackedClose= 0;

        // 决策是否插入关键帧
        // Thresholds
        // 设定比例阈值，当前帧和参考关键帧跟踪到点的比例，比例越大，越倾向于增加关键帧
        float thRefRatio = 0.75f;
        // 关键帧少，那么插入关键帧的阈值设置的低一点，插入频率较低
        if(nKFs<3)
            thRefRatio = 0.4f;

        /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
        const int thStereoClosedPoints = 15;
        if(nClosedPoints < thStereoClosedPoints && (mSensor==System::STEREO || mSensor==System::IMU_STEREO))
        {
            //Pseudo-monocular, there are not enough close points to be confident about the stereo observations.
            thRefRatio = 0.9f;
        }*/

        // 单目情况下插入关键帧的频率很高 TODO 感觉对于这种效果差的可以适当降低一点,这里将其注释掉
//    if(mSensor==System::MONOCULAR)
//        thRefRatio = 0.9f;

        if(mSensor==System::MONOCULAR_EMT)
        {
            if(mnMatchesInliers>350) // Points tracked from the local map
                thRefRatio = 0.75f;
            else
                thRefRatio = 0.90f;
        }

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        // 很长时间没有插入关键帧，可以插入
        const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        // 满足插入关键帧的最小间隔，可以插入
        const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames); //mpLocalMapper->KeyframesInQueue() < 2);
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        // 和参考帧相比当前跟踪到的点太少 或者满足bNeedToInsertClose；同时跟踪到的内点还不能太少
        const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio) && mnMatchesInliers>15);

        // Temporal condition for Inertial cases
        bool c3 = false;
        if(mpLastKeyFrame)
        {
            if (mSensor==System::MONOCULAR_EMT)
            {
                if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                    c3 = true;
            }
        }

        bool c4 = false;
        if ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && (mSensor == System::MONOCULAR_EMT)) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
            c4=true;
        else
            c4=false;

        if(((c1a||c1b) && c2)||c3 ||c4)
        {
            // If the mapping accepts keyframes, insert keyframe.
            // Otherwise send a signal to interrupt BA
            // 空闲可以直接插入
            return true;
        }
        else
            return false;

    }

    void Tracking::CreateNewKeyFrame()
    {
        KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap());

        mpReferenceKF = pKF;
        mCurrentFrame.mpReferenceKF = pKF;

        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKF;
    }

    void Tracking::SetStepByStep(bool bSet)
    {
        bStepByStep = bSet;
    }

    bool Tracking::GetStepByStep()
    {
        return bStepByStep;
    }
}