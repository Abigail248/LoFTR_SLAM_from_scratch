/**
  ******************************************************************************
  * @file           : Settings.h
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
 **/


#ifndef SETTINGS_H
#define SETTINGS_H

#include "CameraModels/GeometricCamera.h"
#include "CameraModels/Pinhole.h"
#include "CameraModels/KannalaBrandt8.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>


namespace LoFTR_SLAM {

    class System;

    //TODO: change to double instead of float

    class Settings {
    public:
        /*
         * Enum for the different camera types implemented
         */
        enum CameraType {
            PinHole = 0,
            Rectified = 1,
            KannalaBrandt = 2
        };

        /**
         * Delete default constructor
         */
        Settings() = delete;
        /**
         * Constructor from file
         */
        Settings(const std::string &configFile, const int& sensor);
        /**
         * Ostream operator overloading to dump settings to the terminal
         */
        friend std::ostream &operator<<(std::ostream &output, const Settings &s);

        /**
         * Getter methods
         */
        CameraType cameraType() {return cameraType_;}
        GeometricCamera* camera() {return calibration_;}
        cv::Mat cameraDistortionCoef() {return cv::Mat(vPinHoleDistorsion_.size(),1,CV_32F,vPinHoleDistorsion_.data());}

        bool needToUndistort() {return bNeedToUndistort_;}

        cv::Size newImSize() {return newImSize_;}
        float fps() {return fps_;}
        bool rgb() {return bRGB_;}
        bool needToResize() {return bNeedToResize_;}

        bool insertKFsWhenLost() {return insertKFsWhenLost_;}

        float depthMapFactor() {return depthMapFactor_;}

        float keyFrameSize() {return keyFrameSize_;}
        float keyFrameLineWidth() {return keyFrameLineWidth_;}
        float graphLineWidth() {return graphLineWidth_;}
        float pointSize() {return pointSize_;}
        float cameraSize() {return cameraSize_;}
        float cameraLineWidth() {return cameraLineWidth_;}
        float viewPointX() {return viewPointX_;}
        float viewPointY() {return viewPointY_;}
        float viewPointZ() {return viewPointZ_;}
        float viewPointF() {return viewPointF_;}
        float imageViewerScale() {return imageViewerScale_;}

        float thFarPoints() {return thFarPoints_;}

        cv::Mat M1l() {return M1l_;}
        cv::Mat M2l() {return M2l_;}
        cv::Mat M1r() {return M1r_;}
        cv::Mat M2r() {return M2r_;}

    private:
        template<typename T>
        T readParameter(cv::FileStorage& fSettings, const std::string& name, bool& found,const bool required = true){
            cv::FileNode node = fSettings[name];
            if(node.empty()){
                if(required){
                    std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                    exit(-1);
                }
                else{
                    std::cerr << name << " optional parameter does not exist..." << std::endl;
                    found = false;
                    return T();
                }

            }
            else{
                found = true;
                return (T) node;
            }
        }

        void readCamera(cv::FileStorage& fSettings);
        void readImageInfo(cv::FileStorage& fSettings);
        void readViewer(cv::FileStorage& fSettings);
        void readOtherParameters(cv::FileStorage& fSettings);

        int sensor_;
        CameraType cameraType_;     //Camera type

        GeometricCamera *calibration_;   //Camera calibration
        GeometricCamera *originalCalib_;
        std::vector<float> vPinHoleDistorsion_;

        cv::Size originalImSize_, newImSize_;
        float fps_;
        bool bRGB_;

        bool bNeedToUndistort_;
        bool bNeedToResize_;

        /*
         * Rectification stuff
         */
        cv::Mat M1l_, M2l_;
        cv::Mat M1r_, M2r_;


        bool insertKFsWhenLost_;

        /*
         * RGBD stuff
         */
        float depthMapFactor_;

        /*
         * Viewer stuff
         */
        float keyFrameSize_;
        float keyFrameLineWidth_;
        float graphLineWidth_;
        float pointSize_;
        float cameraSize_;
        float cameraLineWidth_;
        float viewPointX_, viewPointY_, viewPointZ_, viewPointF_;
        float imageViewerScale_;


        /*
         * Other stuff
         */
        float thFarPoints_;

    };
};

#endif //SETTINGS_H
