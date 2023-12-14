/**
  ******************************************************************************
  * @file           : Optimizer.h
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/13
  ******************************************************************************
 **/



#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"

#include <math.h>

#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace LoFTR_SLAM
{

    class Optimizer
    {
    public:

        /**
         * 3D-2D 最小化重投影误差 e = (u, v) - project(Tcw * Pw)
         * Vertex: g2o::VertexSE3Expmap()，即当前帧的Tcw
         *         g2o::VertexSBAPointXYZ()，MapPoint的mWorldPos
         * Edge: g2o::EdgeSE3ProjectXYZ()，BaseBinaryEdge
         *            Vertex：待优化当前帧的Tcw
         *            Vertex：待优化MapPoint的mWorldPos
         *            measurement：MapPoint在当前帧中的二维位置(u,v)
         * @param vpKF 关键帧
         * @param vpMP MapPoints
         * @param nIterations 迭代次数
         * @param pbStopFlag 是否强制暂停
         * @param bRobust 是否使用核函数
         */
        void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                     int nIterations = 5, bool *pbStopFlag=NULL, const bool bRobust = true);
        /**
         * 进行全局BA优化，但主要功能还是调用 BundleAdjustment,这个函数相当于加了一个壳.
         * @param pMap 地图对象的指针
         * @param nIterations 迭代次数
         * @param pbStopFlag  外界给的控制GBA停止的标志位
         * @param bRobust 是否使用鲁棒核函数
         */
        void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                           const bool bRobust = true);
        /**
         * @brief Local Bundle Adjustment
         *
         * 1. Vertex:
         *     - g2o::VertexSE3Expmap()，LocalKeyFrames，即当前关键帧的位姿、与当前关键帧相连的关键帧的位姿
         *     - g2o::VertexSE3Expmap()，FixedCameras，即能观测到LocalMapPoints的关键帧（并且不属于LocalKeyFrames）的位姿，在优化中这些关键帧的位姿不变
         *     - g2o::VertexSBAPointXYZ()，LocalMapPoints，即LocalKeyFrames能观测到的所有MapPoints的位置
         * 2. Edge:
         *     - g2o::EdgeSE3ProjectXYZ()，BaseBinaryEdge
         *         + Vertex：关键帧的Tcw，MapPoint的Pw
         *         + measurement：MapPoint在关键帧中的二维位置(u,v)
         *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
         *     - g2o::EdgeStereoSE3ProjectXYZ()，BaseBinaryEdge
         *         + Vertex：关键帧的Tcw，MapPoint的Pw
         *         + measurement：MapPoint在关键帧中的二维位置(ul,v,ur)
         *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
         *
         * @param pKF        KeyFrame
         * @param pbStopFlag 是否停止优化的标志
         * @param pMap       在优化后，更新状态时需要用到Map的互斥量mMutexMapUpdate
         * @note 由局部建图线程调用,对局部地图进行优化的函数
         */
        void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges);

        /**
         * @brief Pose Only Optimization
         *
         * 3D-2D 最小化重投影误差 e = (u,v) - project(Tcw*Pw) \n
         * 只优化Frame的Tcw，不优化MapPoints的坐标
         *
         * 1. Vertex: g2o::VertexSE3Expmap()，即当前帧的Tcw
         * 2. Edge:
         *     - g2o::EdgeSE3ProjectXYZOnlyPose()，BaseUnaryEdge
         *         + Vertex：待优化当前帧的Tcw
         *         + measurement：MapPoint在当前帧中的二维位置(u,v)
         *     - g2o::EdgeStereoSE3ProjectXYZOnlyPose()，BaseUnaryEdge
         *         + Vertex：待优化当前帧的Tcw
         *         + measurement：MapPoint在当前帧中的二维位置(ul,v,ur)
         *
         * @param   pFrame Frame
         * @return  inliers数量
         */
        int static PoseOptimization(Frame* pFrame);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

}

#endif //OPTIMIZER_H
