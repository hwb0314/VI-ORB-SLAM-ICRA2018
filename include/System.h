/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

//#include "Tracking.h"
//#include "FrameDrawer.h"
//#include "MapDrawer.h"
#include "Map.h"
//#include "LocalMapping.h"
//#include "LoopClosing.h"
//#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
//#include "Viewer.h"

#include "../src/IMU/imudata.h"
//#include "../src/IMU/configparam.h"


namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class KeyFrameDatabase;
class MapDrawer;

class ConfigParam;  /// for VI-ORB_SLAM

//class IMUData;

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2//,
//        MONOCULAR_VIO=3
    };

/********************************************************************************/
/**************************** for VI-ORB_SLAM2 Start ****************************/
/********************************************************************************/

public:
    bool bLocalMapAcceptKF(void);
    bool bLocalMapperCreateNewMapPointsFinished(void);

    bool SetConfigParam(ConfigParam* pParams);

    // Proccess the given monocular frame and IMU data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonoVI(const cv::Mat &im, const std::vector<IMUData> &vimu, const double &timestamp);
    bool GetMonoVIEnable(void);
    void SetMonoVIEnable(bool flag=false);
    bool GetDeactiveLoopCloserInMonoVI(void);
    void SetDeactiveLoopCloserInMonoVI(bool flag=false);

    void SaveKeyFrameTrajectoryNavState(const string &filename);

    // retrun the state of track with IMU.
    bool GetTrackWithIMUState(void);
//    bool GetTrackingState(void);

    // return the time-consuming of ProcessingFrame, construct Frame, Track, TrackWithIMU, TrackLocalMapWithIMU
    double GetTimeOfProcessingFrame(void);
    double GetTimeOfConstructFrame(void);
    double GetTimeOfTrack(void);
    double GetTimeOfTrackWithIMU(void);
    double GetTimeOfTrackLocalMapWithIMU(void);

    // return the time-consuming of ORB Extract (in Tracking.h,  in ORBextractor.cc)
    double GetTimeOfComputePyramid(void);
    double GetTimeOfComputeKeyPointsOctTree(void);
    double GetTImeOfComputeDescriptor(void);

    // return the time-consuming of LocalMapping
    double GetTimeOfProcessNewKeyFrame(void);
    double GetTimeOfComputeBow(void);
    double GetTimeOfAssociateMapPoints(void);
    double GetTimeOfUpdateConnections(void);
    double GetTimeOfMapPointCulling(void);
    double GetTimeOfCreateNewMapPoints(void);
    double GetTimeOfSearchInNeighbors(void);
    double GetTimeOfLocalBA(void);
    double GetTimeOfKeyFrameCulling(void);


private:
    bool mbMonoVIEnable;
    bool mbDeactiveLoopCloserInMonoVI;

    ConfigParam* mpParams;
/********************************************************************************/
/***************************** for VI-ORB_SLAM2 End *****************************/
/********************************************************************************/

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear map)
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

private:

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;
};

}// namespace ORB_SLAM

//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta 品红*/
#define CYAN "\033[36m" /* Cyan 青色 */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

/*
 * 在构造函数中记录当前的系统时间，在析构函数中输出当前的系统时间与之前的差，精度是us
 * 使用方法：在需要计时的程序段之前构造类对象，在程序段之后获取时间
 * example:
 *		 Timer time; //开始计时
 *		 ....
 *		 printf("time: %d us\n", time.runTime()); //显示时间

*/
// 计时
#include <stdio.h>
#include <sys/time.h>
class Timer
{
    public:

        struct timeval start, end;
        Timer() // 构造函数，开始记录时间
        {
            gettimeofday( &start, NULL );
        }
        void freshTimer()
        {
            gettimeofday( &start, NULL );
        }

        int runTime()
        {
            gettimeofday( &end, NULL );
            return 1000000 * ( end.tv_sec - start.tv_sec ) + end.tv_usec -start.tv_usec;
        }
        double runTime_us()
        {
            return runTime()/1.0;
        }
        double runTime_ms()
        {
            return runTime() / 1000.0;
        }
        double runTime_s()
        {
            return runTime() / 1000000.0;
        }
};

#endif // SYSTEM_H
