/**
  ******************************************************************************
  * @file           : KeyFrame.cpp
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/13
  ******************************************************************************
 **/

#include "KeyFrame.h"

namespace LoFTR_SLAM
{
    long unsigned int KeyFrame::nNextId=0;

    KeyFrame::KeyFrame():
            mnFrameId(0),  mTimeStamp(0), mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), fx(0), fy(0), cx(0), cy(0),
            invfx(0), invfy(0), N(0), mvKeys(static_cast<vector<cv::KeyPoint> >(NULL)),
            mvKeysUn(static_cast<vector<cv::KeyPoint> >(NULL)), mnMinX(0), mnMinY(0), mnMaxX(0), mnMaxY(0),
            mbNotErase(false), mbToBeErased(false), mbBad(false),mnNumberOfOpt(0), mbFirstConnection(false)
    {

    }

    KeyFrame::KeyFrame(Frame &F, Map *pMap):
            mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnTrackReferenceForFrame(0), mnFuseTargetForKF(0),
            fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
            N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
            mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
            mnMaxY(F.mnMaxY), mK_(F.mK_),  mvpMapPoints(F.mvpMapPoints),
            mDistCoef(F.mDistCoef), mbNotErase(false), mbFirstConnection(false),
            mbToBeErased(false), mbBad(false), mpMap(pMap),
            mpCamera(F.mpCamera), mImGray(F.mImGray), mnNumberOfOpt(0)
    {
        mnId=nNextId++;

        SetPose(F.GetPose());

        mnOriginMapId = pMap->GetId();
    }

    void KeyFrame::SetPose(const Sophus::SE3f &Tcw)
    {
        unique_lock<mutex> lock(mMutexPose);

        mTcw = Tcw;
        mRcw = mTcw.rotationMatrix();
        mTwc = mTcw.inverse();
        mRwc = mTwc.rotationMatrix();
    }

    Sophus::SE3f KeyFrame::GetPose()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mTcw;
    }

    Sophus::SE3f KeyFrame::GetPoseInverse()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mTwc;
    }

    Eigen::Vector3f KeyFrame::GetCameraCenter()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mTwc.translation();
    }

    Eigen::Matrix3f KeyFrame::GetRotation()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mRcw;
    }

    Eigen::Vector3f KeyFrame::GetTranslation()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mTcw.translation();
    }

    void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
    {
        {
            // 增加连接，更新权重
            unique_lock<mutex> lock(mMutexConnections);
            if(!mConnectedKeyFrameWeights.count(pKF))
                mConnectedKeyFrameWeights[pKF]=weight;
            else if(mConnectedKeyFrameWeights[pKF]!=weight)
                mConnectedKeyFrameWeights[pKF]=weight;
            else
                return;
        }

        // 如果添加了更新的连接关系就要更新一下,主要是重新进行排序
        UpdateBestCovisibles();
    }

    void KeyFrame::EraseConnection(KeyFrame *pKF)
    {

        bool bUpdate = false;
        {
            unique_lock<mutex> lock(mMutexConnections);
            if(mConnectedKeyFrameWeights.count(pKF))
            {
                mConnectedKeyFrameWeights.erase(pKF);
                bUpdate=true;
            }
        }

        if(bUpdate)
            // 如删除了连接关系就要更新一下,主要是重新进行排序
            UpdateBestCovisibles();
    }

    void KeyFrame::UpdateConnections(bool upParent)
    {
        // 1. 首先获得该关键帧的所有MapPoint点，统计观测到这些3d点的每个关键帧与其它所有关键帧之间的共视程度
        //    对每一个找到的关键帧，建立一条边，边的权重是该关键帧与当前关键帧公共3d点的个数。
        // 2. 并且该权重必须大于一个阈值，如果没有超过该阈值的权重，那么就只保留权重最大的边（与其它关键帧的共视程度比较高）
        // 3. 对这些连接按照权重从大到小进行排序，以方便将来的处理
        //    更新完covisibility图之后，如果没有初始化过，则初始化为连接权重最大的边（与其它关键帧共视程度最高的那个关键帧），类似于最大生成树

        map<KeyFrame*,int> KFcounter;  // 关键帧-权重，权重为其它关键帧与当前关键帧共视3d点的个数

        vector<MapPoint*> vpMP;

        {
            // 获得该关键帧的所有3D点
            unique_lock<mutex> lockMPs(mMutexFeatures);
            vpMP = mvpMapPoints;
        }

        //For all map points in keyframe check in which other keyframes are they seen
        //Increase counter for those keyframes
        // 通过3D点间接统计可以观测到这些3D点的所有关键帧之间的共视程度
        // Step 1 统计每一个地图点都有多少关键帧与当前关键帧存在共视关系，统计结果放在KFcounter
        for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;

            if(!pMP)
                continue;

            if(pMP->isBad())
                continue;

            // 对于每一个MapPoint点，observations 记录了所有可以观测到MapPoint的关键帧
            map<KeyFrame*, int> observations = pMP->GetObservations();

            for(map<KeyFrame*,int>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
            {
                // 自己与自己不算共视
                if(mit->first->mnId==mnId || mit->first->isBad() || mit->first->GetMap() != mpMap)
                    continue;
                KFcounter[mit->first]++;

            }
        }

        // This should not happen
        if(KFcounter.empty())
            return;

        //If the counter is greater than threshold add connection
        //In case no keyframe counter is over threshold add the one with maximum counter
        int nmax=0;
        KeyFrame* pKFmax=NULL;
        // 至少有15个共识点
        int th = 15;

        // vPairs记录与其它关键帧共视帧数大于th的关键帧
        // pair<int,KeyFrame*>将关键帧的权重写在前面，关键帧写在后面方便后面排序
        vector<pair<int,KeyFrame*> > vPairs;
        vPairs.reserve(KFcounter.size());
        if(!upParent)
            cout << "UPDATE_CONN: current KF " << mnId << endl;
        for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
        {
            if(!upParent)
                cout << "  UPDATE_CONN: KF " << mit->first->mnId << " ; num matches: " << mit->second << endl;
            if(mit->second>nmax)
            {
                nmax=mit->second;
                pKFmax=mit->first;
            }
            if(mit->second>=th)
            {
                vPairs.push_back(make_pair(mit->second,mit->first));
                // 对方关键帧也要添加这个信息
                (mit->first)->AddConnection(this,mit->second);
            }
        }

        //  如果没有连接到关键（超过阈值的权重），则对权重最大的关键帧建立连接
        if(vPairs.empty())
        {
            vPairs.push_back(make_pair(nmax,pKFmax));
            pKFmax->AddConnection(this,nmax);
        }

        // 对共视程度比较高的关键帧对更新连接关系及权重（从大到小）
        sort(vPairs.begin(),vPairs.end());
        list<KeyFrame*> lKFs;
        list<int> lWs;
        for(size_t i=0; i<vPairs.size();i++)
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        {
            unique_lock<mutex> lockCon(mMutexConnections);

            mConnectedKeyFrameWeights = KFcounter;
            mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
            mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

            // 更新生成树的连接
            if(mbFirstConnection && mnId!=mpMap->GetInitKFid())
            {
                // 初始化该关键帧的父关键帧为共视程度最高的那个关键帧
                mpParent = mvpOrderedConnectedKeyFrames.front();
                // 建立双向连接关系，将当前关键帧作为其子关键帧
                mpParent->AddChild(this);
                mbFirstConnection = false;
            }

        }
    }

    void KeyFrame::UpdateBestCovisibles()
    {
        unique_lock<mutex> lock(mMutexConnections);

        vector<pair<int,KeyFrame*> > vPairs;
        vPairs.reserve(mConnectedKeyFrameWeights.size());

        for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
            vPairs.push_back(make_pair(mit->second,mit->first));

        sort(vPairs.begin(),vPairs.end());
        list<KeyFrame*> lKFs;
        list<int> lWs;
        for(size_t i=0, iend=vPairs.size(); i<iend;i++)
        {
            if(!vPairs[i].second->isBad())
            {
                lKFs.push_front(vPairs[i].second);
                lWs.push_front(vPairs[i].first);
            }
        }

        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
    }

    std::set<KeyFrame *> KeyFrame::GetConnectedKeyFrames()
    {
        unique_lock<mutex> lock(mMutexConnections);
        set<KeyFrame*> s;
        for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
            s.insert(mit->first);
        return s;
    }

    std::vector<KeyFrame *> KeyFrame::GetVectorCovisibleKeyFrames()
    {
        unique_lock<mutex> lock(mMutexConnections);
        return mvpOrderedConnectedKeyFrames;
    }

    std::vector<KeyFrame *> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
    {
        unique_lock<mutex> lock(mMutexConnections);
        if((int)mvpOrderedConnectedKeyFrames.size()<N)
            return mvpOrderedConnectedKeyFrames;
        else
            return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);
    }

    std::vector<KeyFrame *> KeyFrame::GetCovisiblesByWeight(const int &w)
    {
        unique_lock<mutex> lock(mMutexConnections);

        if(mvpOrderedConnectedKeyFrames.empty())
        {
            return vector<KeyFrame*>();
        }

        vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);

        if(it==mvOrderedWeights.end() && mvOrderedWeights.back() < w)
        {
            return vector<KeyFrame*>();
        }
        else
        {
            int n = it-mvOrderedWeights.begin();
            return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
        }
    }

    int KeyFrame::GetWeight(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
            return mConnectedKeyFrameWeights[pKF];
        else
            return 0;
    }

    void KeyFrame::AddChild(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.insert(pKF);
    }

    void KeyFrame::EraseChild(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.erase(pKF);
    }

    void KeyFrame::ChangeParent(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        if(pKF == this)
        {
            cout << "ERROR: Change parent KF, the parent and child are the same KF" << endl;
            throw std::invalid_argument("The parent and child can not be the same");
        }

        mpParent = pKF;
        pKF->AddChild(this);
    }

    set<KeyFrame*> KeyFrame::GetChilds()
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens;
    }

    KeyFrame* KeyFrame::GetParent()
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mpParent;
    }

    bool KeyFrame::hasChild(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens.count(pKF);
    }

    void KeyFrame::SetFirstConnection(bool bFirst)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mbFirstConnection=bFirst;
    }

    int KeyFrame::GetNumberMPs()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        int numberMPs = 0;
        for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
        {
            if(!mvpMapPoints[i])
                continue;
            numberMPs++;
        }
        return numberMPs;
    }

    void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx]=pMP;
    }

    void KeyFrame::EraseMapPointMatch(const int &idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
    }

    void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
    {
        size_t index = pMP->GetIndexInKeyFrame(this);
        if(index != -1)
            mvpMapPoints[index]=static_cast<MapPoint*>(NULL);
    }

    void KeyFrame::ReplaceMapPointMatch(const int &idx, MapPoint* pMP)
    {
        mvpMapPoints[idx]=pMP;
    }

    set<MapPoint*> KeyFrame::GetMapPoints()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        set<MapPoint*> s;
        for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
        {
            if(!mvpMapPoints[i])
                continue;
            MapPoint* pMP = mvpMapPoints[i];
            if(!pMP->isBad())
                s.insert(pMP);
        }
        return s;
    }

    int KeyFrame::TrackedMapPoints(const int &minObs)
    {
        unique_lock<mutex> lock(mMutexFeatures);

        int nPoints=0;
        const bool bCheckObs = minObs>0;
        for(int i=0; i<N; i++)
        {
            MapPoint* pMP = mvpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(bCheckObs)
                    {
                        if(mvpMapPoints[i]->Observations()>=minObs)
                            nPoints++;
                    }
                    else
                        nPoints++;
                }
            }
        }

        return nPoints;
    }

    vector<MapPoint*> KeyFrame::GetMapPointMatches()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints;
    }

    MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints[idx];
    }

    bool KeyFrame::IsInImage(const float &x, const float &y) const
    {
        return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
    }

    void KeyFrame::SetNotErase()
    {
        unique_lock<mutex> lock(mMutexConnections);
        mbNotErase = true;
    }

    void KeyFrame::SetErase()
    {
        if(mbToBeErased)
        {
            SetBadFlag();
        }
    }

    void KeyFrame::SetBadFlag()
    {
        // 首先处理一下删除不了的特殊情况
        {
            unique_lock<mutex> lock(mMutexConnections);
            // 第0关键帧不允许删除
            if(mnId==mpMap->GetInitKFid())
            {
                return;
            }
            // mbNotErase 表示不能进行删除
            else if(mbNotErase)
            {
                mbToBeErased = true;
                return;
            }
        }
        // 遍历所有和当前关键帧共视的关键帧，删除他们与当前关键帧的联系
        for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        {
            mit->first->EraseConnection(this);
        }

        // 遍历每一个当前关键帧的地图点，删除每一个地图点和当前关键帧的联系
        for(size_t i=0; i<mvpMapPoints.size(); i++)
        {
            if(mvpMapPoints[i])
            {
                mvpMapPoints[i]->EraseObservation(this);
            }
        }

        {
            unique_lock<mutex> lock(mMutexConnections);
            unique_lock<mutex> lock1(mMutexFeatures);

            // 清空自己与其它关键帧之间的联系
            mConnectedKeyFrameWeights.clear();
            mvpOrderedConnectedKeyFrames.clear();

            // Update Spanning Tree
            // 更新生成树，主要是处理好父子关键帧，不然会造成整个关键帧维护的图断裂，或者混乱，不能够为后端提供较好的初值
            // 子关键帧候选父关键帧
            set<KeyFrame*> sParentCandidates;
            if(mpParent)// 将当前帧的父关键帧放入候选父关键帧
                sParentCandidates.insert(mpParent);

            // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
            // Include that children as new parent candidate for the rest
            // 如果这个关键帧有自己的子关键帧，告诉这些子关键帧，它们的父关键帧不行了，赶紧找新的父关键帧
            while(!mspChildrens.empty())
            {
                bool bContinue = false;

                int max = -1;
                KeyFrame* pC;
                KeyFrame* pP;

                for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
                {
                    KeyFrame* pKF = *sit;
                    if(pKF->isBad())
                        continue;

                    // Check if a parent candidate is connected to the keyframe
                    vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                    for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                    {
                        for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                        {
                            if(vpConnected[i]->mnId == (*spcit)->mnId)
                            {
                                int w = pKF->GetWeight(vpConnected[i]);
                                if(w>max)
                                {
                                    pC = pKF;
                                    pP = vpConnected[i];
                                    max = w;
                                    bContinue = true;
                                }
                            }
                        }
                    }
                }

                if(bContinue)
                {
                    pC->ChangeParent(pP);
                    sParentCandidates.insert(pC);
                    mspChildrens.erase(pC);
                }
                else
                    break;
            }

            // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
            if(!mspChildrens.empty())
            {
                for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
                {
                    (*sit)->ChangeParent(mpParent);
                }
            }

            if(mpParent){
                mpParent->EraseChild(this);
                mTcp = mTcw * mpParent->GetPoseInverse();
            }
            mbBad = true;
        }
        mpMap->EraseKeyFrame(this);
    }

    bool KeyFrame::isBad()
    {
        unique_lock<mutex> lock(mMutexConnections);
        return mbBad;
    }

    float KeyFrame::ComputeSceneMedianDepth(const int q)
    {
        if(N==0)
            return -1.0;

        vector<MapPoint*> vpMapPoints;
        Eigen::Matrix3f Rcw;
        Eigen::Vector3f tcw;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPose);
            vpMapPoints = mvpMapPoints;
            tcw = mTcw.translation();
            Rcw = mRcw;
        }

        vector<float> vDepths;
        vDepths.reserve(N);
        Eigen::Matrix<float,1,3> Rcw2 = Rcw.row(2);
        float zcw = tcw(2);
        for(int i=0; i<N; i++) {
            if(mvpMapPoints[i])
            {
                MapPoint* pMP = mvpMapPoints[i];
                Eigen::Vector3f x3Dw = pMP->GetWorldPos();
                float z = Rcw2.dot(x3Dw) + zcw;
                vDepths.push_back(z);
            }
        }

        sort(vDepths.begin(),vDepths.end());

        return vDepths[(vDepths.size()-1)/q];
    }

    Map* KeyFrame::GetMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void KeyFrame::UpdateMap(Map* pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }
}