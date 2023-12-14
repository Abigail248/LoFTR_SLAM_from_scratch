/**
  ******************************************************************************
  * @file           : Frame.cpp
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/13
  ******************************************************************************
 **/

#include "Frame.h"

namespace LoFTR_SLAM
{

    long unsigned int Frame::nNextId=0;
    bool Frame::mbInitialComputations = true;
    float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
    float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;

    Frame::Frame(): mpPrevFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbHasPose(false), mbIsSet(false)
    {
#ifdef REGISTER_TIMES
        mTimeStereoMatch = 0;
    mTimeORB_Ext = 0;
#endif
    }

    Frame::Frame(const Frame &frame)
            :mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mK_(Converter::toMatrix3f(frame.mK)),
             mDistCoef(frame.mDistCoef.clone()), N(frame.N), mvKeys(frame.mvKeys),
             mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier),
             mnId(frame.mnId), mpReferenceKF(frame.mpReferenceKF), mpPrevFrame(frame.mpPrevFrame),
             mpLastKeyFrame(frame.mpLastKeyFrame), mpCamera(frame.mpCamera), mbIsSet(frame.mbIsSet),
             mTcw(frame.mTcw), mbHasPose(false), mImGray(frame.mImGray)
    {
        if(frame.mbHasPose)
            SetPose(frame.GetPose());


#ifdef REGISTER_TIMES
        mTimeStereoMatch = frame.mTimeStereoMatch;
    mTimeORB_Ext = frame.mTimeORB_Ext;
#endif
    }

    Frame::Frame(const cv::Mat &imGray, const double &timeStamp, GeometricCamera* pCamera,
                 cv::Mat &distCoef, Frame* pPrevF):mImGray(imGray), mTimeStamp(timeStamp),
                 mK(static_cast<Pinhole*>(pCamera)->toK()), mbIsSet(false),
                 mK_(static_cast<Pinhole*>(pCamera)->toK_()), mDistCoef(distCoef.clone()), N(0),
                 mpPrevFrame(pPrevF), mpReferenceKF(static_cast<KeyFrame*>(NULL)),
                 mpCamera(pCamera), mbHasPose(false)
    {
        // Frame ID
        mnId=nNextId++;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif
        // 为特征点和地图点预留5000大小的空间
        mvKeys.reserve(5000);
        mvKeysUn.reserve(5000);
        mvpMapPoints.reserve(5000);
        mvbOutlier.reserve(5000);

        if(mbInitialComputations)
        {
            ComputeImageBounds(imGray);

            fx = static_cast<Pinhole*>(mpCamera)->toK().at<float>(0,0);
            fy = static_cast<Pinhole*>(mpCamera)->toK().at<float>(1,1);
            cx = static_cast<Pinhole*>(mpCamera)->toK().at<float>(0,2);
            cy = static_cast<Pinhole*>(mpCamera)->toK().at<float>(1,2);
            invfx = 1.0f/fx;
            invfy = 1.0f/fy;

            mbInitialComputations=false;
        }


    }


    void Frame::SetPose(const Sophus::SE3<float> &Tcw)
    {
        mTcw = Tcw;

        UpdatePoseMatrices();
        mbIsSet = true;
        mbHasPose = true;
    }

    void Frame::UpdatePoseMatrices()
    {
        Sophus::SE3<float> Twc = mTcw.inverse();
        mRwc = Twc.rotationMatrix();
        mOw = Twc.translation();
        mRcw = mTcw.rotationMatrix();
        mtcw = mTcw.translation();
    }

    bool Frame::isSet() const
    {
        return mbIsSet;
    }

    bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
    {
        // Step 1 获得这个地图点的世界坐标
        // Step 2 关卡一：检查这个地图点在当前帧的相机坐标系下，是否有正的深度.如果是负的，表示出错，返回false
        // Step 3 关卡二：将MapPoint投影到当前帧的像素坐标(u,v), 并判断是否在图像有效范围内
        // Step 4 关卡三：计算当前相机指向地图点向量和地图点的平均观测方向夹角的余弦值, 若小于设定阈值，返回false
        // Step 5 记录计算得到的一些参数
        pMP->mbTrackInView = false;
        pMP->mTrackProjX = -1;
        pMP->mTrackProjY = -1;

        // 3D in absolute coordinates
        Eigen::Matrix<float,3,1> P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const Eigen::Matrix<float,3,1> Pc = mRcw * P + mtcw;
        const float Pc_dist = Pc.norm();

        // Check positive depth
        const float &PcZ = Pc(2);
        const float invz = 1.0f/PcZ;
        if(PcZ<0.0f)
            return false;

        const Eigen::Vector2f uv = mpCamera->project(Pc);

        if(uv(0)<mnMinX || uv(0)>mnMaxX)
            return false;
        if(uv(1)<mnMinY || uv(1)>mnMaxY)
            return false;

        pMP->mTrackProjX = uv(0);
        pMP->mTrackProjY = uv(1);

        // Check viewing angle
        const Eigen::Vector3f PO = P - mOw;
        const float dist = PO.norm();
        Eigen::Vector3f Pn = pMP->GetNormal();

        const float viewCos = PO.dot(Pn)/dist;

        if(viewCos<viewingCosLimit)
            return false;

        // Data used by the tracking
        pMP->mbTrackInView = true;
        pMP->mTrackProjX = uv(0);

        pMP->mTrackDepth = Pc_dist;

        pMP->mTrackProjY = uv(1);
        pMP->mTrackViewCos = viewCos;

        return true;
    }

    void Frame::UndistortKeyPoints(int start) {
        if(mDistCoef.at<float>(0)==0.0)
        {
            mvKeysUn=mvKeys;
            return;
        }

        // Fill matrix with points
        cv::Mat mat(N-start,2,CV_32F);

        for(int i=start; i<N; i++)
        {
            mat.at<float>(i,0)=mvKeys[i].pt.x;
            mat.at<float>(i,1)=mvKeys[i].pt.y;
        }

        // Undistort points
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat, static_cast<Pinhole*>(mpCamera)->toK(),mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);


        // Fill undistorted keypoint vector
        mvKeysUn.resize(N);
        for(int i=start; i<N; i++)
        {
            cv::KeyPoint kp = mvKeys[i];
            kp.pt.x=mat.at<float>(i,0);
            kp.pt.y=mat.at<float>(i,1);
            mvKeysUn[i]=kp;
        }
    }

    void Frame::ComputeImageBounds(const cv::Mat &im) {
        if(mDistCoef.at<float>(0)!=0.0)
        {
            cv::Mat mat(4,2,CV_32F);
            mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
            mat.at<float>(1,0)=im.cols; mat.at<float>(1,1)=0.0;
            mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=im.rows;
            mat.at<float>(3,0)=im.cols; mat.at<float>(3,1)=im.rows;

            mat=mat.reshape(2);
            cv::undistortPoints(mat,mat,static_cast<Pinhole*>(mpCamera)->toK(),mDistCoef,cv::Mat(),mK);
            mat=mat.reshape(1);

            // Undistort corners
            mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
            mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
            mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
            mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));
        }
        else
        {
            mnMinX = 0.0f;
            mnMaxX = im.cols;
            mnMinY = 0.0f;
            mnMaxY = im.rows;
        }
    }

    void Frame::UpdateKeyPoint(vector<cv::KeyPoint> &vkps, vector<tuple<int, int>> &vtnMatchIndexes, int flag) {
        // indexes前面是index
        if (vkps.empty())
            return;

        // mvKeys为空
        if (this->mvKeys.empty())
        {
            std::copy(vkps.begin(), vkps.end(), std::back_inserter(this->mvKeys));
        }
        else
        {
            int count = 0;
            int index = -1;
            // 如果keypoint不存在，那么加入mvkeys
            // 判断keypoint是不是同一个的依据是像素点位置
            for (int i=0; i<mvKeys.size(); i++)
            {
                for (int j=0; j<vkps.size(); j++)
                {
                    if (mvKeys[i].pt.x == vkps[j].pt.x && mvKeys[i].pt.y == vkps[j].pt.y)
                    {
                        index = i;
                    }
                    else
                    {
                        mvKeys.push_back(vkps[j]);
                        index = mvKeys.size() + count;
                        count++;
                    }

                    if (flag == 0)
                    {
                        get<0>(vtnMatchIndexes[j]) = index;
                    }
                    else
                    {
                        get<1>(vtnMatchIndexes[j]) = index;
                    }
                }
            }
        }

        // 重新修改相关vector的大小
        int start = N;
        N = mvKeys.size();

        UndistortKeyPoints(start);

        if (start == 0)
        {
            mvpMapPoints = vector<MapPoint*>(N, static_cast<MapPoint*>(NULL));
            mvbOutlier = vector<bool>(N, false);
        }
        else
        {
            mvpMapPoints.resize(N, static_cast<MapPoint*>(NULL));
            mvbOutlier.resize(N, false);
        }
    }

}