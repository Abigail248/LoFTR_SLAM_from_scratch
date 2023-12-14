/**
  ******************************************************************************
  * @file           : Settings.cpp
  * @author         : abigail
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
 **/

#include "Settings.h"

#include "CameraModels/Pinhole.h"
#include "CameraModels/KannalaBrandt8.h"

#include "System.h"

#include <opencv2/core/persistence.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>

using namespace std;

namespace LoFTR_SLAM{

    template<>
    float Settings::readParameter<float>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if(node.empty()){
            if(required){
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else{
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return 0.0f;
            }
        }
        else if(!node.isReal()){
            std::cerr << name << " parameter must be a real number, aborting..." << std::endl;
            exit(-1);
        }
        else{
            found = true;
            return node.real();
        }
    }

    template<>
    int Settings::readParameter<int>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if(node.empty()){
            if(required){
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else{
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return 0;
            }
        }
        else if(!node.isInt()){
            std::cerr << name << " parameter must be an integer number, aborting..." << std::endl;
            exit(-1);
        }
        else{
            found = true;
            return node.operator int();
        }
    }

    template<>
    string Settings::readParameter<string>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if(node.empty()){
            if(required){
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else{
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return string();
            }
        }
        else if(!node.isString()){
            std::cerr << name << " parameter must be a string, aborting..." << std::endl;
            exit(-1);
        }
        else{
            found = true;
            return node.string();
        }
    }

    template<>
    cv::Mat Settings::readParameter<cv::Mat>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if(node.empty()){
            if(required){
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else{
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return cv::Mat();
            }
        }
        else{
            found = true;
            return node.mat();
        }
    }

    Settings::Settings(const string &configFile, const int &sensor) :
        bNeedToUndistort_(false), bNeedToResize_(false)
    {
        sensor_ = sensor;

        //Open settings file
        cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
        if (!fSettings.isOpened())
        {
            cerr << "[ERROR]: could not open configuration file at: " << configFile << endl;
            cerr << "Aborting..." << endl;

            exit(-1);
        }
        else
        {
            cout << "Loading settings from " << configFile << endl;
        }

        //Read camera
        readCamera(fSettings);
        cout << "\t-Loaded camera" << endl;

        //Read image info
        readImageInfo(fSettings);
        cout << "\t-Loaded image info" << endl;

        //Read Viewer info
        readViewer(fSettings);
        cout << "\t-Loaded viewer settings" << endl;

        cout << "----------------------------------" << endl;
    }

    void Settings::readCamera(cv::FileStorage &fSettings) {
        bool found;

        //Read camera model
        string cameraModel = readParameter<string>(fSettings,"Camera.type",found);

        vector<float> vCalibration;
        if (cameraModel == "PinHole") {
            cameraType_ = PinHole;

            //Read intrinsic parameters
            float fx = readParameter<float>(fSettings,"Camera1.fx",found);
            float fy = readParameter<float>(fSettings,"Camera1.fy",found);
            float cx = readParameter<float>(fSettings,"Camera1.cx",found);
            float cy = readParameter<float>(fSettings,"Camera1.cy",found);

            vCalibration = {fx, fy, cx, cy};

            calibration_ = new Pinhole(vCalibration);
            originalCalib_ = new Pinhole(vCalibration);

            //Check if it is a distorted PinHole
            readParameter<float>(fSettings,"Camera.k1",found,false);
            if(found){
                readParameter<float>(fSettings,"Camera.k3",found,false);
                if(found){
                    vPinHoleDistorsion_.resize(5);
                    vPinHoleDistorsion_[4] = readParameter<float>(fSettings,"Camera.k3",found);
                }
                else{
                    vPinHoleDistorsion_.resize(4);
                }
                vPinHoleDistorsion_[0] = readParameter<float>(fSettings,"Camera.k1",found);
                vPinHoleDistorsion_[1] = readParameter<float>(fSettings,"Camera.k2",found);
                vPinHoleDistorsion_[2] = readParameter<float>(fSettings,"Camera.p1",found);
                vPinHoleDistorsion_[3] = readParameter<float>(fSettings,"Camera.p2",found);
            }

            //Check if we need to correct distortion from the images
            if(vPinHoleDistorsion_.size() != 0){
                bNeedToUndistort_ = true;
            }
        }
        else if(cameraModel == "Rectified"){
            cameraType_ = Rectified;

            //Read intrinsic parameters
            float fx = readParameter<float>(fSettings,"Camera.fx",found);
            float fy = readParameter<float>(fSettings,"Camera.fy",found);
            float cx = readParameter<float>(fSettings,"Camera.cx",found);
            float cy = readParameter<float>(fSettings,"Camera.cy",found);

            vCalibration = {fx, fy, cx, cy};

            calibration_ = new Pinhole(vCalibration);
            originalCalib_ = new Pinhole(vCalibration);

            //Rectified images are assumed to be ideal PinHole images (no distortion)
        }
        else if(cameraModel == "KannalaBrandt8"){
            cameraType_ = KannalaBrandt;

            //Read intrinsic parameters
            float fx = readParameter<float>(fSettings,"Camera.fx",found);
            float fy = readParameter<float>(fSettings,"Camera.fy",found);
            float cx = readParameter<float>(fSettings,"Camera.cx",found);
            float cy = readParameter<float>(fSettings,"Camera.cy",found);

            float k0 = readParameter<float>(fSettings,"Camera.k1",found);
            float k1 = readParameter<float>(fSettings,"Camera.k2",found);
            float k2 = readParameter<float>(fSettings,"Camera.k3",found);
            float k3 = readParameter<float>(fSettings,"Camera.k4",found);

            vCalibration = {fx,fy,cx,cy,k0,k1,k2,k3};

            calibration_ = new KannalaBrandt8(vCalibration);
            originalCalib_ = new KannalaBrandt8(vCalibration);
        }
        else{
            cerr << "Error: " << cameraModel << " not known" << endl;
            exit(-1);
        }
    }

    void Settings::readImageInfo(cv::FileStorage &fSettings)
    {
        bool found;
        //Read original and desired image dimensions
        int originalRows = readParameter<int>(fSettings,"Camera.height",found);
        int originalCols = readParameter<int>(fSettings,"Camera.width",found);
        originalImSize_.width = originalCols;
        originalImSize_.height = originalRows;

        newImSize_ = originalImSize_;
        int newHeigh = readParameter<int>(fSettings,"Camera.newHeight",found,false);
        if(found){
            bNeedToResize_ = true;
            newImSize_.height = newHeigh;

            //Update calibration
            float scaleRowFactor = (float)newImSize_.height / (float)originalImSize_.height;
            calibration_->setParameter(calibration_->getParameter(1) * scaleRowFactor, 1);
            calibration_->setParameter(calibration_->getParameter(3) * scaleRowFactor, 3);
        }
        int newWidth = readParameter<int>(fSettings,"Camera.newWidth",found,false);
        if(found){
            bNeedToResize_ = true;
            newImSize_.width = newWidth;

            //Update calibration
            float scaleColFactor = (float)newImSize_.width /(float) originalImSize_.width;
            calibration_->setParameter(calibration_->getParameter(0) * scaleColFactor, 0);
            calibration_->setParameter(calibration_->getParameter(2) * scaleColFactor, 2);
        }

        fps_ = readParameter<int>(fSettings,"Camera.fps",found);
        bRGB_ = (bool) readParameter<int>(fSettings,"Camera.RGB",found);
    }

    void Settings::readViewer(cv::FileStorage &fSettings) {
        bool found;

        keyFrameSize_ = readParameter<float>(fSettings,"Viewer.KeyFrameSize",found);
        keyFrameLineWidth_ = readParameter<float>(fSettings,"Viewer.KeyFrameLineWidth",found);
        graphLineWidth_ = readParameter<float>(fSettings,"Viewer.GraphLineWidth",found);
        pointSize_ = readParameter<float>(fSettings,"Viewer.PointSize",found);
        cameraSize_ = readParameter<float>(fSettings,"Viewer.CameraSize",found);
        cameraLineWidth_ = readParameter<float>(fSettings,"Viewer.CameraLineWidth",found);
        viewPointX_ = readParameter<float>(fSettings,"Viewer.ViewpointX",found);
        viewPointY_ = readParameter<float>(fSettings,"Viewer.ViewpointY",found);
        viewPointZ_ = readParameter<float>(fSettings,"Viewer.ViewpointZ",found);
        viewPointF_ = readParameter<float>(fSettings,"Viewer.ViewpointF",found);
        imageViewerScale_ = readParameter<float>(fSettings,"Viewer.imageViewScale",found,false);

        if(!found)
            imageViewerScale_ = 1.0f;
    }

    void Settings::readOtherParameters(cv::FileStorage& fSettings) {
        bool found;

        thFarPoints_ = readParameter<float>(fSettings,"System.thFarPoints",found,false);
    }

    ostream &operator<<(std::ostream& output, const Settings& settings){
        output << "SLAM settings: " << endl;

        output << "\t-Camera parameters (";
        if(settings.cameraType_ == Settings::PinHole || settings.cameraType_ ==  Settings::Rectified){
            output << "Pinhole";
        }
        else{
            output << "Kannala-Brandt";
        }
        output << ")" << ": [";
        for(size_t i = 0; i < settings.originalCalib_->size(); i++){
            output << " " << settings.originalCalib_->getParameter(i);
        }
        output << " ]" << endl;

        if(!settings.vPinHoleDistorsion_.empty()){
            output << "\t-Camera 1 distortion parameters: [ ";
            for(float d : settings.vPinHoleDistorsion_){
                output << " " << d;
            }
            output << " ]" << endl;
        }

        output << "\t-Original image size: [ " << settings.originalImSize_.width << " , " << settings.originalImSize_.height << " ]" << endl;
        output << "\t-Current image size: [ " << settings.newImSize_.width << " , " << settings.newImSize_.height << " ]" << endl;


        if(settings.bNeedToResize_){
            output << "\t-Camera 1 parameters after resize: [ ";
            for(size_t i = 0; i < settings.calibration_->size(); i++){
                output << " " << settings.calibration_->getParameter(i);
            }
            output << " ]" << endl;
        }

        output << "\t-Sequence FPS: " << settings.fps_ << endl;

        return output;
    }



}