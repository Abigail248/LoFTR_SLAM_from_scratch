/**
  ******************************************************************************
  * @file           : TwoViewReconstruction.h
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
 **/



#ifndef TWOVIEWRECONSTRUCTION_H
#define TWOVIEWRECONSTRUCTION_H


#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <unordered_set>

#include <sophus/se3.hpp>

namespace LoFTR_SLAM
{

    class TwoViewReconstruction
    {
        typedef std::pair<int,int> Match;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // Fix the reference frame
        TwoViewReconstruction(const Eigen::Matrix3f& k, float sigma = 1.0, int iterations = 200);

        // Computes in parallel a fundamental matrix and a homography
        // Selects a model and tries to recover the motion and the structure from motion
        bool Reconstruct(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
                         Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated);
        bool ReconstructEmt(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
                            Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated);

    private:

        void FindHomography(std::vector<bool> &vbMatchesInliers, float &score, Eigen::Matrix3f &H21);
        void FindFundamental(std::vector<bool> &vbInliers, float &score, Eigen::Matrix3f &F21);

        Eigen::Matrix3f ComputeH21(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);
        Eigen::Matrix3f ComputeF21(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);

        float CheckHomography(const Eigen::Matrix3f &H21, const Eigen::Matrix3f &H12, std::vector<bool> &vbMatchesInliers, float sigma);

        float CheckFundamental(const Eigen::Matrix3f &F21, std::vector<bool> &vbMatchesInliers, float sigma);

        bool ReconstructF(std::vector<bool> &vbMatchesInliers, Eigen::Matrix3f &F21, Eigen::Matrix3f &K,
                          Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        bool ReconstructH(std::vector<bool> &vbMatchesInliers, Eigen::Matrix3f &H21, Eigen::Matrix3f &K,
                          Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D,std:: vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        void Normalize(const std::vector<cv::KeyPoint> &vKeys, std::vector<cv::Point2f> &vNormalizedPoints, Eigen::Matrix3f &T);


        int CheckRT(const Eigen::Matrix3f &R, const Eigen::Vector3f &t, const std::vector<cv::KeyPoint> &vKeys1, const std::vector<cv::KeyPoint> &vKeys2,
                    const std::vector<Match> &vMatches12, std::vector<bool> &vbMatchesInliers,
                    const Eigen::Matrix3f &K, std::vector<cv::Point3f> &vP3D, float th2, std::vector<bool> &vbGood, float &parallax);

        void DecomposeE(const Eigen::Matrix3f &E, Eigen::Matrix3f &R1, Eigen::Matrix3f &R2, Eigen::Vector3f &t);


        // Keypoints from Reference Frame (Frame 1)
        std::vector<cv::KeyPoint> mvKeys1;

        // Keypoints from Current Frame (Frame 2)
        std::vector<cv::KeyPoint> mvKeys2;

        // Current Matches from Reference to Current
        std::vector<Match> mvMatches12;
        std::vector<bool> mvbMatched1;

        // Calibration
        Eigen::Matrix3f mK;

        // Standard Deviation and Variance
        float mSigma, mSigma2;

        // Ransac max iterations
        int mMaxIterations;

        // Ransac sets
        std::vector<std::vector<size_t> > mvSets;

    };

} //namespace LoFTR_SLAM

#endif TWOVIEWRECONSTRUCTION_H