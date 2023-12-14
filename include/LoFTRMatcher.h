/**
  ******************************************************************************
  * @file           : LoFTRMatcher.h
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
 **/


#ifndef LoFTRMatcher_H
#define LoFTRMatcher_H

#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"

#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;

namespace LoFTR_SLAM{
    class LoFTRMatcher{
    public:
        LoFTRMatcher(const string &modelPath, const int &useCUDA=1, const int &threadNum=8);

        ~LoFTRMatcher(){}

        // Matching for the Map Initialization (Only for monocular case)
        int SearchForInitialization(Frame &F1, Frame &F2, vector<int> &vnMatches12);

        int SearchForTrack(KeyFrame* pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);

    protected:
        void imageProcess(cv::Mat &image);


        int match(const cv::Mat &image0, const cv::Mat &image1, vector<cv::KeyPoint> &kps1, vector<cv::KeyPoint> &kps2, const float confTh=0.5);

        Ort::Env mEnv{nullptr};
        Ort::SessionOptions mSessionOptions{nullptr};
        Ort::Session mSession{nullptr};

        vector<int64_t> mvInputDims;
        size_t mNumInputNodes, mNumOutputNodes;
        size_t mInputTensorSize;
        vector<char*> mvInputNames;
        vector<char*> mvOutputNames;
    };


}

#endif //LoFTRMatcher_H
