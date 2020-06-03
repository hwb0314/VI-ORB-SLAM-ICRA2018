#include "configparam.h"

namespace ORB_SLAM2
{
double ConfigParam::_g = 9.810;

Eigen::Matrix4d ConfigParam::_EigTbc = Eigen::Matrix4d::Identity();
cv::Mat ConfigParam::_MatTbc = cv::Mat::eye(4,4,CV_32F);
Eigen::Matrix4d ConfigParam::_EigTcb = Eigen::Matrix4d::Identity();
cv::Mat ConfigParam::_MatTcb = cv::Mat::eye(4,4,CV_32F);
int ConfigParam::_LocalWindowSize = 10;
double ConfigParam::_ImageDelayToIMU = 0;
bool ConfigParam::_bAccMultiply9p8 = false;
std::string ConfigParam::_tmpFilePath = "";

ConfigParam::ConfigParam(std::string configfile)
{
    cv::FileStorage fSettings(configfile, cv::FileStorage::READ);

    std::cout<<std::endl<<std::endl<<"Parameters: "<<std::endl;

    _testDiscardTime = fSettings["test.DiscardTime"];

    fSettings["test.InitVIOTmpPath"] >> _tmpFilePath;
    std::cout<<"save tmp file in "<<_tmpFilePath<<std::endl;

    fSettings["bagfile"] >> _bagfile;
    std::cout<<"open rosbag: "<<_bagfile<<std::endl;
    fSettings["imutopic"] >> _imuTopic;
    fSettings["imagetopic"] >> _imageTopic;
    std::cout<<"imu topic: "<<_imuTopic<<std::endl;
    std::cout<<"image topic: "<<_imageTopic<<std::endl;

    _LocalWindowSize = fSettings["LocalMapping.LocalWindowSize"];
    std::cout<<"local window size: "<<_LocalWindowSize<<std::endl;

    _ImageDelayToIMU = fSettings["Camera.delaytoimu"];
    std::cout<<"timestamp image delay to imu: "<<_ImageDelayToIMU<<std::endl;

    {
        cv::FileNode Tbc_ = fSettings["Camera.Tbc"];
        Eigen::Matrix<double,3,3> R;
        R << Tbc_[0], Tbc_[1], Tbc_[2],
                Tbc_[4], Tbc_[5], Tbc_[6],
                Tbc_[8], Tbc_[9], Tbc_[10];
        Eigen::Quaterniond qr(R);
        R = qr.normalized().toRotationMatrix();
        Eigen::Matrix<double,3,1> t( Tbc_[3], Tbc_[7], Tbc_[11]);
        _EigTbc = Eigen::Matrix4d::Identity();
        _EigTbc.block<3,3>(0,0) = R;
        _EigTbc.block<3,1>(0,3) = t;
        _MatTbc = cv::Mat::eye(4,4,CV_32F);
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                _MatTbc.at<float>(i,j) = _EigTbc(i,j);

        _EigTcb = Eigen::Matrix4d::Identity();
        _EigTcb.block<3,3>(0,0) = R.transpose();
        _EigTcb.block<3,1>(0,3) = -R.transpose()*t;
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                _MatTcb.at<float>(i,j) = _EigTcb(i,j);

        // Tbc_[0], Tbc_[1], Tbc_[2], Tbc_[3], Tbc_[4], Tbc_[5], Tbc_[6], Tbc_[7], Tbc_[8], Tbc_[9], Tbc_[10], Tbc_[11], Tbc_[12], Tbc_[13], Tbc_[14], Tbc_[15];
        std::cout<<"Tbc inited:"<<std::endl<<_EigTbc<<std::endl<<_MatTbc<<std::endl;
        std::cout<<"Tcb inited:"<<std::endl<<_EigTcb<<std::endl<<_MatTcb<<std::endl;
        std::cout<<"Tbc*Tcb:"<<std::endl<<_EigTbc*_EigTcb<<std::endl<<_MatTbc*_MatTcb<<std::endl;
    }

    {
        int tmpBool = fSettings["IMU.multiplyG"];
        _bAccMultiply9p8 = (tmpBool != 0);
        std::cout<<"whether acc*9.8? 0/1: "<<_bAccMultiply9p8<<std::endl;
    }


    {
        _iImuRate = fSettings["IMU.rate"];
        _dGyroMeaSigma_g = fSettings["IMU.sigma_g"];
        _dGyroBiasSigma_gw = fSettings["IMU.sigma_gw"];
        _dAccMeaSigma_a = fSettings["IMU.sigma_a"];
        _dAccBiasSigma_aw = fSettings["IMU.sigma_aw"];

        std::cout<<"Imu Rate: " << _iImuRate << std::endl;
        std::cout<<"GyroMeasureNoiseSigma: " << _dGyroMeaSigma_g << std::endl;
        std::cout<<"GyroBiasNoiseSigma: " << _dGyroBiasSigma_gw << std::endl;
        std::cout<<"AccMeasureNoiseSigma: " << _dAccMeaSigma_a << std::endl;
        std::cout<<"AccBiasNoiseSigma: " << _dAccBiasSigma_aw << std::endl;
    }

    {
        _runningMode = fSettings["runningMode"];
        if (_runningMode == 1) std::cout<<"runningMode: " << "MonoVI" << std::endl;
        if (_runningMode == 0) std::cout<<"runningMode: " << "Monocular" << std::endl;

        int tmpDeactiveLoopClosure = fSettings["deactiveLoopClosure"];
        _bDeactiveLoopClosure = (tmpDeactiveLoopClosure!=0);
        if (_bDeactiveLoopClosure) std::cout<< "\033[32m" << "deactiveLoopClosure: True" << "\033[0m" << std::endl;
        if (!_bDeactiveLoopClosure) std::cout<<"deactiveLoopClosure: False" << std::endl;

        int tmpDispalyTimeStatistic = fSettings["dispalyTimeStatistic"];
        _bDispalyTimeStatistic = (tmpDispalyTimeStatistic==1);
        if(_bDispalyTimeStatistic) std::cout << "dispalyTimeStatistic: True" << std::endl;
        if(!_bDispalyTimeStatistic) std::cout << "dispalyTimeStatistic: False" << std::endl;

        int tmpVisionAidWhenTrackWithIMUFail = fSettings["visionAidWhenTrackWithIMUFail"];
        _bVisionAidWhenTrackWithIMUFail = (tmpVisionAidWhenTrackWithIMUFail==1);
        if(_bVisionAidWhenTrackWithIMUFail) std::cout << "\033[31m" << "visionAidWhenTrackWithIMUFail: True" << "\033[0m" << std::endl;
        if(!_bVisionAidWhenTrackWithIMUFail) std::cout << "visionAidWhenTrackWithIMUFail: False" << std::endl;

        int tmpWaitUntilLocalMapIdle = fSettings["waitUntilLocalMapIdle"];
        _bWaitUntilLocalMapIdle = (tmpWaitUntilLocalMapIdle==1);
        if(_bWaitUntilLocalMapIdle) std::cout << "\033[31m" << "waitUntilLocalMapIdle: True" << "\033[0m" << std::endl;
        if(!_bWaitUntilLocalMapIdle) std::cout << "waitUntilLocalMapIdle: False" << std::endl;

        int tmpOnlyTrackLocalMap = fSettings["onlyTrackLocalMap"];
        _bOnlyTrackLocalMap = (tmpOnlyTrackLocalMap==1);
        if(_bOnlyTrackLocalMap) std::cout << "\033[31m" << "onlyTrackLocalMap: True" << "\033[0m" << std::endl;
        if(!_bOnlyTrackLocalMap) std::cout << "onlyTrackLocalMap: False" << std::endl;

        int tmpEstimateExtrinsicBetweenCameraAndIMU = fSettings["estimateExtrinsicBetweenCameraAndIMU"];
        _bEstimateExtrinsicBetweenCameraAndIMU = (tmpEstimateExtrinsicBetweenCameraAndIMU==1);
        if(_bEstimateExtrinsicBetweenCameraAndIMU) std::cout << "\033[31m" << "estimateExtrinsicBetweenCameraAndIMU: True" << "\033[0m" << std::endl;
        if(!_bEstimateExtrinsicBetweenCameraAndIMU) std::cout << "estimateExtrinsicBetweenCameraAndIMU: False (Use the pre-calibrated Tbc)" << std::endl;

        int tmpCreateRandomMatrixLastForFirstVIOInit = fSettings["CreateRandomMatrixLastForFirstVIOInit"];
        _bCreateRandomMatrixLastForFirstVIOInit = (tmpCreateRandomMatrixLastForFirstVIOInit==1);
        if(_bCreateRandomMatrixLastForFirstVIOInit) std::cout << "\033[31m" << "CreateRandomMatrixLastForFirstVIOInit: True" << "\033[0m" << std::endl;
        if(!_bCreateRandomMatrixLastForFirstVIOInit) std::cout << "CreateRandomMatrixLastForFirstVIOInit: False (Use the Identity Matrix for Rbcstar_last)" << std::endl;
    }
}

std::string ConfigParam::getTmpFilePath()
{
    return _tmpFilePath;
}

int ConfigParam::GetImuRate()
{
    return _iImuRate;
}

double ConfigParam::GetGyroMeasureNoiseSigma()
{
    return _dGyroMeaSigma_g;
}

double ConfigParam::GetGyroBiasNoiseSigma()
{
    return _dGyroBiasSigma_gw;
}

double ConfigParam::GetAccMeasureNoiseSigma()
{
    return _dAccMeaSigma_a;
}

double ConfigParam::GetAccBiasNoiseSigma()
{
    return _dAccBiasSigma_aw;
}

Eigen::Matrix4d ConfigParam::GetEigTbc()
{
    return _EigTbc;
}

cv::Mat ConfigParam::GetMatTbc()
{
    return _MatTbc.clone();
}

Eigen::Matrix4d ConfigParam::GetEigT_cb()
{
    return _EigTcb;
}

cv::Mat ConfigParam::GetMatT_cb()
{
    return _MatTcb.clone();
}

int ConfigParam::GetLocalWindowSize()
{
    return _LocalWindowSize;
}

double ConfigParam::GetImageDelayToIMU()
{
    return _ImageDelayToIMU;
}

bool ConfigParam::GetAccMultiply9p8()
{
    return _bAccMultiply9p8;
}

int ConfigParam::GetRunningMode()
{
    return _runningMode;
}

bool ConfigParam::GetDeactiveLoopClosure()
{
    return _bDeactiveLoopClosure;
}

bool ConfigParam::GetDispalyTimeStatistic()
{
    return _bDispalyTimeStatistic;
}


bool ConfigParam::GetVisionAidWhenTrackWithIMUFail()
{
    return _bVisionAidWhenTrackWithIMUFail;
}

bool ConfigParam::GetWaitUntilLocalMapIdle()
{
    return _bWaitUntilLocalMapIdle;
}

bool ConfigParam::GetOnlyTrackLocalMap()
{
    return _bOnlyTrackLocalMap;
}

bool ConfigParam::GetEstimateExtrinsicBetweenCameraAndIMU()
{
    return _bEstimateExtrinsicBetweenCameraAndIMU;
}

bool ConfigParam::GetCreateRandomMatrixLastForFirstVIOInit()
{
    return _bCreateRandomMatrixLastForFirstVIOInit;
}

}
