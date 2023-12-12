/**
  ******************************************************************************
  * @file           : run_slam.cpp
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
**/
#include<iostream>
#include<algorithm>
#include<chrono>
#include <thread>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc.hpp>

#include <Eigen/Core>

#include<System.h>
#include<Config.h>

using namespace std;

void LoadImages ( const string &strImagePath, const string &strPathTimes,
                  vector<string> &vstrImages, vector<double> &vTimeStamps );
void showTcw(const int index, const double timestamp, const Sophus::SE3f &Tcw);


int main(int argc, char** argv)
{

    if (argc < 6)
    {
        cerr << endl << "Usage: ./Mono images_dir_path image_time_path voc_path LoFTR_model_path (mask_image_path) setting_path" << endl;
        return 1;
    }

    bool bFileName = 0;


    // Load Configuration file
    string strSettingPath = string (argv[argc-1]);
    string strImagePath = string (argv[1]);
    string strPathTimes = string (argv[2]);
    string strVocPath   = string (argv[3]);
    string strLoFTRModelPath = string(argv[4]);
    string strMaskImagePath = string ();
    if (argc == 7)
    {
        strMaskImagePath = argv[argc-2];
    }

    vector<string> vstrImages;
    vector<double> vTimeStampsCam;
    vector<Eigen::Vector3f > vPosition;
    vector<Eigen::Vector4f> vDirections;
    int nImages;

    // Load data
    cout << "Load Images ... " << endl;
    LoadImages(strImagePath, strPathTimes, vstrImages, vTimeStampsCam);
    cout << "LOADED!" << endl;

    nImages = static_cast<int>(vstrImages.size());


    ORB_SLAM3::System SLAM (strVocPath, strMaskImagePath, strSettingPath, strLoFTRModelPath, ORB_SLAM3::System::MONOCULAR, false);


    float imageScale = SLAM.GetImageScale();

    cv::Mat im;
    vector<ORB_SLAM3::EM::Point> vEmMeas;

    for ( int ni=0 ; ni<nImages; ni++ )
    {
        // Read image from file
        im = cv::imread(vstrImages[ni], cv::IMREAD_UNCHANGED);

        double tframe = vTimeStampsCam[ni];

        if ( im.empty() )
        {
            cerr << endl <<  "Failed to load image at: " << vstrImages[ni].c_str() << endl;
            return 1;
        }else
        {
            cerr << endl << "Processing image: " << vstrImages[ni].c_str() << endl;
        }

        if ( imageScale != 1.f ) {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        SLAM.TrackMonocular(im, tframe);
//        SLAM.Initialization(im, tframe , "");
    }
    SLAM.Shutdown();

#ifndef VERSION_1
    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
#endif

}

void LoadImages ( const string &strImagePath, const string &strPathTimes,
                  vector<string> &vstrImages, vector<double> &vTimeStamps )
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    // 设置空间大小为 5000
    vstrImages.reserve(5000);
    vTimeStamps.reserve(5000);
    string timeStamp, imageName;
    while (!fTimes.eof()) {
        fTimes >> timeStamp >> imageName;
        vTimeStamps.push_back(stod(timeStamp));
        vstrImages.push_back(strImagePath + "/" + imageName);
    }
}

void showTcw(const int index, const double timestamp, const Sophus::SE3f &Tcw)
{
    cout << index << " " << timestamp << " ";
    cout << Tcw.unit_quaternion().x() << " " << Tcw.unit_quaternion().y() << " " << Tcw.unit_quaternion().z() << " " <<
         Tcw.unit_quaternion().w() << " ";
    cout << Tcw.translation().x() << " " << Tcw.translation().y() << " " << Tcw.translation().z() << endl;
}

