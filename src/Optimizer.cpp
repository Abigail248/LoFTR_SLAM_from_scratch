/**
  ******************************************************************************
  * @file           : Optimizer.cpp
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/14
  ******************************************************************************
 **/

#include "Optimizer.h"

#include <complex>
#include<mutex>

#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <g2o/core/sparse_block_matrix.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include "Converter.h"
#include "System.h"

namespace LoFTR_SLAM
{

    bool sortByVal(const pair<MapPoint*, int> &a, const pair<MapPoint*, int> &b)
    {
        return (a.second < b.second);
    }

    void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKF, const vector<MapPoint *> &vpMP, int nIterations,
                                     bool *pbStopFlag, const bool bRobust)
    {

        // 不参与优化的地图点
        vector<bool> vbNotIncludedMP;
        vbNotIncludedMP.resize(vpMP.size());

        Map* pMap = vpKF[0]->GetMap();

        // 1. 初始化g2o优化器
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3( std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> (linearSolver));

        // 使用 LM 算法进行优化
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<g2o::BlockSolver_6_3> (solver_ptr) );
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        // 如果这个时候外部请求终止，那就结束
        // 执行完这句之后，如果外部再次请求结束BA，那就结束不了了
        if(pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        // 记录添加到优化器中的顶点中的最大关键帧id
        long unsigned int maxKFid = 0;

        const int nExpectedSize = (vpKF.size())*vpMP.size();

        // 定义顶点并预留空间
        vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        // 定义关键帧之间的边并预留空间
        vector<KeyFrame*> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        // 定义顶点的边
        vector<MapPoint*> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        // 2. 向优化器添加顶点
        // Set KeyFrame vertices
        // 向优化器添加关键帧位姿顶点
        for(size_t i=0; i<vpKF.size(); i++)
        {
            KeyFrame* pKF = vpKF[i];
            // 去除无效的关键帧
            if(pKF->isBad())
                continue;

            // 对于每个能用关键帧构造的SE3顶点，其实就是当前关键帧的位姿
            g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
            Sophus::SE3<float> Tcw = pKF->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));
            vSE3->setId(pKF->mnId);
            // 只有初始关键帧不进行优化
            vSE3->setFixed(pKF->mnId==pMap->GetInitKFid());

            // 向优化器中添加顶点，并且更新maxKFid
            optimizer.addVertex(vSE3);
            if(pKF->mnId>maxKFid)
                maxKFid=pKF->mnId;
        }
        // 卡方分布 95% 以上可信度的时候的阈值
        const float thHuber2D = sqrt(5.99);     // 自由度为2
        const float thHuber3D = sqrt(7.815);     // 自由度为3

        // Set MapPoint vertices
        // 向优化器中添加MapPoint顶点
        for(size_t i=0; i<vpMP.size(); i++)
        {
            MapPoint* pMP = vpMP[i];
            if(pMP->isBad())
                continue;
            g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
            const int id = pMP->mnId+maxKFid+1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame*, int> observations = pMP->GetObservations();

            // 边计数
            int nEdges = 0;
            //SET EDGES
            // 3. 向优化器添加投影边
            for(map<KeyFrame*, int>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
            {
                KeyFrame* pKF = mit->first;
                if(pKF->isBad() || pKF->mnId>maxKFid)
                    continue;
                if(optimizer.vertex(id) == NULL || optimizer.vertex(pKF->mnId) == NULL)
                    continue;
                nEdges++;

                const int index = mit->second;

                if(index != -1)
                {
                    const cv::KeyPoint &kpUn = pKF->mvKeysUn[index];

                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    // 信息矩阵，也是协方差，表明了这个约束的观测在各个维度（x,y）上的可信程度，
                    // 在我们这里对于具体的一个点，两个坐标的可信程度都是相同的，
                    float invSigma2 = 1/(1.2*1.2);
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    // 使用鲁棒核函数
                    if(bRobust)
                    {
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber2D);
                    }

                    // 设置相机内参
                    e->fx = pKF->fx;
                    e->fy = pKF->fy;
                    e->cx = pKF->cx;
                    e->cy = pKF->cy;

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKF);
                    vpMapPointEdgeMono.push_back(pMP);
                }

            }

            // 如果因为一些特殊原因,实际上并没有任何关键帧观测到当前的这个地图点,那么就删除掉这个顶点,并且这个地图点也就不参与优化
            if(nEdges==0)
            {
                optimizer.removeVertex(vPoint);
                vbNotIncludedMP[i]=true;
            }
            else
            {
                vbNotIncludedMP[i]=false;
            }
        }

        // Optimize!
        // 4.开始优化
        optimizer.setVerbose(false);
        optimizer.initializeOptimization();
        optimizer.optimize(nIterations);
        Verbose::PrintMess("BA: End of the optimization", Verbose::VERBOSITY_NORMAL);

        // Recover optimized data
        // 5. 得到优化结果
        //Keyframes
        for(size_t i=0; i<vpKF.size(); i++)
        {
            KeyFrame* pKF = vpKF[i];
            if(pKF->isBad())
                continue;

            // 获得优化后的位姿
            g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));

            g2o::SE3Quat SE3quat = vSE3->estimate();

            // 这个时候,地图中就只有两个关键帧,其中优化后的位姿数据可以直接写入到帧的成员变量中
            pKF->SetPose(Sophus::SE3f(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>()));
        }

        //Points
        for(size_t i=0; i<vpMP.size(); i++)
        {
            if(vbNotIncludedMP[i])
                continue;

            MapPoint* pMP = vpMP[i];

            if(pMP->isBad())
                continue;
            g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));

            pMP->SetWorldPos(vPoint->estimate().cast<float>());
            pMP->UpdateNormal();
        }
    }

    void Optimizer::GlobalBundleAdjustemnt(Map *pMap, int nIterations, bool *pbStopFlag, const bool bRobust)
    {
        vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
        vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
        BundleAdjustment(vpKFs,vpMP,nIterations,pbStopFlag, bRobust);
    }

    int Optimizer::PoseOptimization(Frame *pFrame)
    {
        // 该优化函数主要用于Tracking线程中：运动跟踪、参考帧跟踪、地图跟踪、重定位

        // Step 1：构造g2o优化器, BlockSolver_6_3表示：位姿 _PoseDim 为6维，路标点 _LandmarkDim 是3维
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> (linearSolver) );

        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<g2o::BlockSolver_6_3> (solver_ptr) );
        optimizer.setAlgorithm(solver);

        // 输入的帧中,有效的,参与优化过程的2D-3D点对
        int nInitialCorrespondences=0;

        // Set Frame vertex
        // Step 2：添加顶点：待优化当前帧的Tcw
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float> Tcw = pFrame->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));
        vSE3->setId(0);
        // 因为是要优化的变量，所以不固定
        vSE3->setFixed(false);
        optimizer.addVertex(vSE3);

        // Set MapPoint vertices
        const int N = pFrame->N;

        vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
        vector<size_t> vnIndexEdgeMono;
        vpEdgesMono.reserve(N);
        vnIndexEdgeMono.reserve(N);

        // 自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值5.991
        const float deltaMono = sqrt(5.991);
        // 自由度为3的卡方分布，显著性水平为0.05，对应的临界阈值7.815
        const float deltaStereo = sqrt(7.815);

        // 添加一元边
        {
            // 锁定地图点。由于需要地图点来构建边所以不希望在优化过程中地图点被改写
            unique_lock<mutex> lock(MapPoint::mGlobalMutex);

            for(int i=0; i<N; i++)
            {
                MapPoint* pMP = pFrame->mvpMapPoints[i];
                if(pMP)
                {
                    nInitialCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double,2,1> obs;
                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = 1 / (1.2 * 1.2);
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);


                    // 设置相机内参
                    e->fx = pFrame->fx;
                    e->fy = pFrame->fy;
                    e->cx = pFrame->cx;
                    e->cy = pFrame->cy;
                    // 地图点的空间位置,作为迭代的初始值
                    e->Xw = pMP->GetWorldPos().cast<double>();

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
            }
        }

        // 如果没有足够的匹配点,那么就只好放弃了
        if(nInitialCorrespondences<3)
            return 0;

        // 开始优化，总共优化四次，每次优化迭代10次,每次优化后，将观测分为outlier和inlier，outlier不参与下次优化
        // 由于每次优化后是对所有的观测进行outlier和inlier判别，因此之前被判别为outlier有可能变成inlier，反之亦然
        // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        const float chi2Mono[4]={5.991,5.991,5.991,5.991};
        const int its[4]={10,10,10,10};

        // bad 的地图点个数
        int nBad=0;

        // 进行四次优化
        for(size_t it=0; it<4; it++)
        {
            Tcw = pFrame->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));

            optimizer.initializeOptimization(0);
            // 开始优化，优化10次
            optimizer.optimize(its[it]);

            nBad=0;
            // 优化结束,开始遍历参与优化的每一条误差边(单目)
            for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
            {
                g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

                const size_t idx = vnIndexEdgeMono[i];

                if(pFrame->mvbOutlier[idx])
                {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if(chi2>chi2Mono[it])
                {
                    pFrame->mvbOutlier[idx]=true;
                    e->setLevel(1);
                    nBad++;
                }
                else
                {
                    pFrame->mvbOutlier[idx]=false;
                    e->setLevel(0);
                }

                if(it==2)
                    e->setRobustKernel(0);
            }

            if(optimizer.edges().size()<10)
                break;
        }

        // Recover optimized pose and return number of inliers
        g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
        g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
        Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
                                SE3quat_recov.translation().cast<float>());
        pFrame->SetPose(pose);

        return nInitialCorrespondences-nBad;
    }
}