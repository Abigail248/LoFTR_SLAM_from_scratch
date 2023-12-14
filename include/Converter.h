/**
  ******************************************************************************
  * @file           : Converter.h
  * @author         : abigail
  * @brief          : 提供了一系列的常见转换。
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
 **/



#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>
#include<Eigen/Dense>

#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <sophus/geometry.hpp>
#include <sophus/sim3.hpp>

namespace LoFTR_SLAM
{
    class Converter
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * 将Mat格式的位姿转换成g2o::SE3Quat格式存储
         * @param cvTranspose cv::Mat格式的位姿
         * @return g2o::SE3Quat格式的位姿
         */
        static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);

        /**
          * 将Sophus::SE3f格式的位姿转换成g2o::SE3Quat格式存储
          * @param T Sophus::SE3f格式的位姿
          * @return g2o::SE3Quat格式的位姿
          */
        static g2o::SE3Quat toSE3Quat(const Sophus::SE3f &T);

        /**
         * 将g2o::Sim3格式的位姿转换成g2o::SE3Quat格式存储
         * @param gSim3 g2o::Sim3格式的位姿
         * @return g2o::SE3Quat格式的位姿
         */
        static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

        // TODO templetize these functions
        /**
         * 将各种模式专程cv::Mat 存储
         * @param
         * @return
         */
        static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
        static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
        static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
        static cv::Mat toCvMat(const Eigen::Matrix<float,4,4> &m);
        static cv::Mat toCvMat(const Eigen::Matrix<float,3,4> &m);
        static cv::Mat toCvMat(const Eigen::Matrix3d &m);
        static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
        static cv::Mat toCvMat(const Eigen::Matrix<float,3,1> &m);
        static cv::Mat toCvMat(const Eigen::Matrix<float,3,3> &m);
        static cv::Mat toCvMat(const Eigen::MatrixXf &m);
        static cv::Mat toCvMat(const Eigen::MatrixXd &m);

        /**
         * @brief 将给定的旋转矩阵和平移向量转换为以cv::Mat存储的李群SE3
         * @details 其实就是组合旋转矩阵和平移向量来构造SE3
         * @param[in] R 旋转矩阵
         * @param[in] t 平移向量
         * @return cv::Mat 李群SE3
         */
        static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);


        static cv::Mat tocvSkewMatrix(const cv::Mat &v);

        static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
        static Eigen::Matrix<float,3,1> toVector3f(const cv::Mat &cvVector);
        static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
        static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
        static Eigen::Matrix<double,4,4> toMatrix4d(const cv::Mat &cvMat4);
        static Eigen::Matrix<float,3,3> toMatrix3f(const cv::Mat &cvMat3);
        static Eigen::Matrix<float,4,4> toMatrix4f(const cv::Mat &cvMat4);
        static std::vector<float> toQuaternion(const cv::Mat &M);

        static bool isRotationMatrix(const cv::Mat &R);
        static std::vector<float> toEuler(const cv::Mat &R);

        //TODO: Sophus migration, to be deleted in the future
        static Sophus::SE3<float> toSophus(const cv::Mat& T);
        static Sophus::Sim3f toSophus(const g2o::Sim3& S);
    };
}

#endif //CONVERTER_H
