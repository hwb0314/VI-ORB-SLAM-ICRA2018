#ifndef CONFIGPARAM_H
#define CONFIGPARAM_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{

class ConfigParam
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConfigParam(std::string configfile);

    double _testDiscardTime;

    static Eigen::Matrix4d GetEigTbc();
    static cv::Mat GetMatTbc();
    static Eigen::Matrix4d GetEigT_cb();
    static cv::Mat GetMatT_cb();
    static int GetLocalWindowSize();
    static double GetImageDelayToIMU();
    static bool GetAccMultiply9p8();

     int GetImuRate();
     double GetGyroMeasureNoiseSigma();
     double GetGyroBiasNoiseSigma();
     double GetAccMeasureNoiseSigma();
     double GetAccBiasNoiseSigma();


    static double GetG(){return _g;}

    int GetRunningMode();
    bool GetDeactiveLoopClosure();

    bool GetDispalyTimeStatistic();
    bool GetVisionAidWhenTrackWithIMUFail();
    bool GetWaitUntilLocalMapIdle();
    bool GetOnlyTrackLocalMap();
    bool GetEstimateExtrinsicBetweenCameraAndIMU();
    bool GetCreateRandomMatrixLastForFirstVIOInit();


    std::string _bagfile;
    std::string _imageTopic;
    std::string _imuTopic;

    static std::string getTmpFilePath();
    static std::string _tmpFilePath;

private:
    static Eigen::Matrix4d _EigTbc;
    static cv::Mat _MatTbc;
    static Eigen::Matrix4d _EigTcb;
    static cv::Mat _MatTcb;
    static int _LocalWindowSize;
    static double _ImageDelayToIMU;
    static bool _bAccMultiply9p8;

    int _iImuRate;
    double _dGyroMeaSigma_g;  // gyro measurement white noise
    double _dGyroBiasSigma_gw; // gyro bias diffusion
    double _dAccMeaSigma_a;  // accel measurement white noise
    double _dAccBiasSigma_aw; // accel bias diffusion

    static double _g;

    int _runningMode;
    bool _bDeactiveLoopClosure;
    bool _bDispalyTimeStatistic;
    bool _bVisionAidWhenTrackWithIMUFail;
    bool _bWaitUntilLocalMapIdle;
    bool _bOnlyTrackLocalMap;
    bool _bEstimateExtrinsicBetweenCameraAndIMU;
    bool _bCreateRandomMatrixLastForFirstVIOInit;
};

}

#endif // CONFIGPARAM_H
