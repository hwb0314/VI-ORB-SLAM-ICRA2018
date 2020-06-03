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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

class LocalMapping
{
/// for VI-ORB_SLAM2
/********************************************************************************/
/**************************** for VI-ORB_SLAM2 Start ****************************/
/********************************************************************************/
public:
    bool GetDeactiveLoopCloserInMonoVI(void);
    void SetDeactiveLoopCloserInMonoVI(bool flag=false);// { mbDeactiveLoopCloserInMonoVI = flag; }

    bool GetMonoVIEnable(void);
    void SetMonoVIEnable(bool flag=false);

    std::mutex mMutexAssociateMapPointConutIncrease;
    void AssociateMapPointsForMultiThread(
            const vector<MapPoint*> vpMapPointMatches,
            size_t &i_count);

    bool bCreateNewMapPointsFinished;

public:
    bool SetConfigParam(ConfigParam* pParams);
//    LocalMapping(Map* pMap, const float bMonocular, ConfigParam* pParams) ;

    // KeyFrames in Local Window, for Local BA
    // Insert in ProcessNewKeyFrame()
    void AddToLocalWindow(KeyFrame* pKF);
    void DeleteBadInLocalWindow(void);

    bool TryInitVIO(void);
    bool TryInitVIOWithoutPreCalibration(void);
    bool GetVINSInited(void);
    void SetVINSInited(bool flag);

    bool GetFirstVINSInited(void);
    void SetFirstVINSInited(bool flag);

    bool GetMapUpdateFlagForTracking();
    void SetMapUpdateFlagInTracking(bool flag);
    bool GetEnableChangeMapUpdateFlagForTracking();
    KeyFrame* GetMapUpdateKF(void);

    void KeyFrameCullingForMonoVI(void);

    double GetVINSInitScale(void);
    cv::Mat GetGravityVec(void);

    cv::Mat GetVINSInitTbc(void);
    cv::Mat GetVINSInitRbc(void);
    cv::Mat GetVINSInitPbc(void);

    void ResetBiasgToZero(std::vector<KeyFrame*> vScaleGravityKF);


    // record the time consuming
    double GetTimeOfProcessNewKeyFrame(void) { return mTimeOfProcessNewKeyFrame; }
    double GetTimeOfComputeBow(void) { return mTimeOfComputeBow; }
    double GetTimeOfAssociateMapPoints(void) { return mTimeOfAssociateMapPoints; }
    double GetTimeOfUpdateConnections(void) { return mTimeOfUpdateConnections; }
    double GetTimeOfMapPointCulling(void) { return mTimeOfMapPointCulling; }
    double GetTimeOfCreateNewMapPoints(void) { return mTimeOfCreateNewMapPoints; }
    double GetTimeOfSearchInNeighbors(void) { return mTimeOfSearchInNeighbors; }
    double GetTimeOfLocalBA(void) { return mTimeOfLocalBA; }
    double GetTimeOfKeyFrameCulling(void) { return mTimeOfKeyFrameCulling; }

    // check convergence
    double GetVariance(std::vector<double> &vector_);           // variance
    double GetStandardDeviation(std::vector<double> &vector_);  // Standard Deviation
    double GetVariance(std::vector<cv::Mat> &vector_);
    bool CheckRbcEstimationConverge();
    bool CheckPbcEstimationConverge();

private:
    ConfigParam* mpParams;
    bool mbMonoVIEnable;
    bool mbDeactiveLoopCloserInMonoVI;

    // record the time consuming
    double mTimeOfProcessNewKeyFrame;
    double mTimeOfComputeBow;
    double mTimeOfAssociateMapPoints;
    double mTimeOfUpdateConnections;
    double mTimeOfMapPointCulling;
    double mTimeOfCreateNewMapPoints;
    double mTimeOfSearchInNeighbors;
    double mTimeOfLocalBA;
    double mTimeOfKeyFrameCulling;

protected:
    double mnStartTime;
    bool mbFirstTry;
    double mnVINSInitScale;     // scale
    cv::Mat mGravityVec;        // gravity vector in world frame
    cv::Mat mVINSInitTbc;       // Tbc
    cv::Mat mVINSInitRbc;       // Rbc
    cv::Mat mVINSInitPbc;       // Pbc
    Vector3d mVINSInitBiasg;    // bgest
    Vector3d mVINSInitBiasa;    // biasa_eig

    bool mbVINSInitRbcConverged;    // Rbc converged
    bool mbVINSInitPbcConverged;    // Pbc converged

    int nNewMapPointsCreatedByLastKF;

    std::mutex mMutexVINSInitFlag;
    bool mbVINSInited;

    std::mutex mMutexFirstVINSInitFlag;
    bool mbFirstVINSInited;

    unsigned int mnLocalWindowSize;
    std::list<KeyFrame*> mlLocalKeyFrames;

    std::mutex mMutexMapUpdateFlag;
    bool mbMapUpdateFlagForTracking;
    bool mbEnableChangeMapUpdateFlagForTracking;    // Enable change mbMapUpdateFlagForTracking

    KeyFrame* mpMapUpdateKF;

    std::vector<Eigen::Vector3d> mvRecordEstimatedRbc;   // Rbc: yaw, roll, pitch
    std::vector<double> mvRecordEstimatedRbcTimeStamps;

    std::vector<Eigen::Vector3d> mvRecordEstimatedPbc;   // Pbc: x,y,z
    std::vector<double> mvRecordEstimatedPbcTimeStamps;

/********************************************************************************/
/***************************** for VI-ORB_SLAM2 End *****************************/
/********************************************************************************/

public:
    LocalMapping(Map* pMap, const float bMonocular);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
