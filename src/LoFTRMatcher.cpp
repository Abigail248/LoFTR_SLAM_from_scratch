/**
  ******************************************************************************
  * @file           : LoFTRMatcher.cpp
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/14
  ******************************************************************************
 **/
#include "LoFTRMatcher.h"
#include <numeric>

using namespace std;
using namespace cv;

template<typename T>
T vectorProduct(const std::vector<T>& v)
{
    return accumulate(v.begin(), v.end(), 1, std::multiplies<T>());
}

namespace LoFTR_SLAM
{

    LoFTRMatcher::LoFTRMatcher(const string &modelPath, const int &useCUDA, const int &threadNum)
    {
        string instanceName{"image-match-inference"};

        // Initialize Environment, one environment per process
        mEnv = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, instanceName.c_str());

        // Initialize session option if needed
        mSessionOptions = Ort::SessionOptions();
        mSessionOptions.SetIntraOpNumThreads(threadNum);

        if (useCUDA == -1){
            cout << "Inference Execution Provider: CPU" << endl;
        }else{
            cout << "Inference Execution Provider: CUDA" << endl;
            OrtSessionOptionsAppendExecutionProvider_CUDA(mSessionOptions, useCUDA);
        }

        // Sets graph optimization level
        // Available levels are
        // ORT_DISABLE_ALL -> To disable all optimizations
        // ORT_ENABLE_BASIC -> To enable basic optimizations (Such as redundant node
        // removals) ORT_ENABLE_EXTENDED -> To enable extended optimizations
        // (Includes level 1 + more complex optimizations like node fusions)
        // ORT_ENABLE_ALL -> To Enable All possible optimizations
        mSessionOptions.SetGraphOptimizationLevel(
                GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        mSession = Ort::Session(mEnv, modelPath.c_str(), mSessionOptions);

        mNumInputNodes = mSession.GetInputCount();
        mNumOutputNodes = mSession.GetOutputCount();

        Ort::AllocatorWithDefaultOptions allocator;
        char* inputName = mSession.GetInputName(0, allocator);

        Ort::TypeInfo inputTypeInfo = mSession.GetInputTypeInfo(0);

        auto TypeTensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
        mvInputDims = TypeTensorInfo.GetShape();

        if (mvInputDims.at(0) == -1){
            mvInputDims.at(0) = 2;
        }

        char * outputName = mSession.GetOutputName(0, allocator);
        mInputTensorSize = vectorProduct(mvInputDims);
        mvInputNames = {inputName};
        mvOutputNames = {outputName};
    }

    void LoFTRMatcher::imageProcess(cv::Mat &image)
    {
        cv::resize(image, image, cv::Size(static_cast<int>(mvInputDims.at(3)),
                                          static_cast<int>(mvInputDims.at(2))), 0, 0, cv::InterpolationFlags::INTER_AREA);
        image.convertTo(image, CV_32F, 1.0/255);
    }

    int LoFTRMatcher::match(const cv::Mat &image0, const cv::Mat &image1,
                            vector<cv::KeyPoint> &kps1, vector<cv::KeyPoint> &kps2, const float confTh)
    {
        int oriW = image0.cols;
        int oriH = image0.rows;
        cv::Mat img0 = image0.clone();
        cv::Mat img1 = image1.clone();

        // resize图像并归一化
        imageProcess(img0);
        imageProcess(img1);

        vector<float> inputTensorsValues(mInputTensorSize);
        copy(img0.begin<float>(), img0.end<float>(),
             inputTensorsValues.begin());
        copy(img1.begin<float>(), img1.end<float>(),
             inputTensorsValues.begin() + mInputTensorSize/2);

        std::vector<Ort::Value> inputTensors;

        Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
                OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault
        );

        inputTensors.push_back(Ort::Value::CreateTensor<float>(
                memoryInfo, inputTensorsValues.data(), mInputTensorSize, mvInputDims.data(), mvInputDims.size()
        ));

        auto outputTensors = mSession.Run(Ort::RunOptions{nullptr}, mvInputNames.data(),
                                          inputTensors.data(), mNumInputNodes,
                                          mvOutputNames.data(), mNumOutputNodes);

        vector<int64_t> outputDims = outputTensors[0].GetTypeInfo().GetTensorTypeAndShapeInfo().GetShape();

        float* floatarr = outputTensors[0].GetTensorMutableData<float>();
        // 第一列为置信度（需要进行归一化）， 后四列分别表示第一张图像的特征点的位置, 第二张图像的特征点的位置
        cv::Mat LoFTRResult = cv::Mat(outputDims[0], outputDims[1], CV_32F, floatarr);

        kps1.reserve(LoFTRResult.rows);
        kps2.reserve(LoFTRResult.rows);

        // 归一化操作获得置信度
        cv::normalize(LoFTRResult.col(0), LoFTRResult.col(0), 1.0, 0.0, cv::NORM_MINMAX, -1, cv::Mat());

        // 计算与原始图像的缩放比例
        float scaleX = static_cast<float>(oriW) / static_cast<float>(mvInputDims.at(3));
        float scaleY = static_cast<float>(oriH) / static_cast<float>(mvInputDims.at(2));

        int nmatches = 0;
        // 从模型结果中获得特征点
        for (int i = 0; i < LoFTRResult.rows; ++i)
        {
            // 过滤掉置信度低于阈值的结果
            if (LoFTRResult.at<float>(i, 0) < confTh)
                continue;
            int x0 = static_cast<int>(LoFTRResult.at<float>(i, 1) * scaleX);
            int y0 = static_cast<int>(LoFTRResult.at<float>(i, 2) * scaleY);
            int x1 = static_cast<int>(LoFTRResult.at<float>(i, 3) * scaleX);
            int y1 = static_cast<int>(LoFTRResult.at<float>(i, 4) * scaleY);

            // 这里将特征点的size设置为1
            kps1.push_back(cv::KeyPoint(x0, y0, 1));
            kps2.push_back(cv::KeyPoint(x1, y1, 1));
            nmatches ++;
        }
        return nmatches;
    }

    int LoFTRMatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<int> &vnMatches12)
    {
        int nmatches = 0;

        vector<cv::KeyPoint> vkps1;
        vector<cv::KeyPoint> vkps2;

        // 进行匹配
        nmatches = match(F1.mImGray, F2.mImGray, vkps1, vkps2);

        vnMatches12 = vector<int>(nmatches, -1);

//        F1.UpdateKeyPoint(vkps1);
//        F2.UpdateKeyPoint(vkps2);

        for(int i=0; i<nmatches; i++)
        {
            vnMatches12[i] = i;
        }

        return nmatches;
    }

    int LoFTRMatcher::SearchForTrack(KeyFrame *pKF, Frame &F, vector<MapPoint *> &vpMapPointMatches) {
        int nmatches = 0;

        int nkeypoint = pKF->mvKeys.size();

        vector<cv::KeyPoint> CurKps;
        vector<cv::KeyPoint> KFKps;

        nmatches = match(pKF->mImGray, F.mImGray, KFKps, CurKps);

        // 保存将新的keypoint插入到F中对应的下标
//        F.UpdateKeyPoint(KFKps);

        vector<int> vnMatches12 = vector<int>(nkeypoint, -1);

        // pFK匹配结果的特征点存在于pKF.mvKeys中，则保存对应的F的关键点，并保存该匹配对
        int index=0;
        for (int i = 0; i < nmatches; ++i) {
            for (int j=0; j<pKF->mvKeys.size(); j++) {
                cv::Point2f kpPoint = pKF->mvKeys[j].pt;
                if (kpPoint.x == CurKps[i].pt.x && kpPoint.y == CurKps[i].pt.y) {
                    vnMatches12[j] = i;
                    index++;
                    break;
                }
            }
        }

        // 获得关键帧的地图点
        const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();

        vpMapPointMatches = vector<MapPoint*>(F.N, static_cast<MapPoint*>(NULL));

        for (int i = 0; i < vnMatches12.size(); ++i) {
            if (vnMatches12[i] == -1)
                break;
            MapPoint* pMP = vpMapPointsKF[i];
            vpMapPointMatches[vnMatches12[i]] = pMP;
        }
        return index;
    }

}

