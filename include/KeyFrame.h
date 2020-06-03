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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>

#include "../src/IMU/NavState.h"
#include "../src/IMU/IMUPreintegrator.h"
#include "../src/IMU/imudata.h"
namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

// Forward Declaration
//class IMUData;


/* KeyFrame
 * 关键帧，和普通的Frame不一样，但是可以由Frame来构造
 * 许多数据会被三个线程同时访问，所以用锁的地方很普遍
 */
class KeyFrame
{
/// for VI-ORB_SLAM2
/********************************************************************************/
/**************************** for VI-ORB_SLAM2 Start ****************************/
/********************************************************************************/
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase *pKFDB, std::vector<IMUData> vIMUData, KeyFrame* pLastKF=NULL);
    void SetIMUData(std::vector<IMUData> vIMUData);
    void SetPrevAndNextKeyFrame(KeyFrame* pPrevKF);

    KeyFrame* GetPrevKeyFrame(void);
    KeyFrame* GetNextKeyFrame(void);
    void SetPrevKeyFrame(KeyFrame* pKF);
    void SetNextKeyFrame(KeyFrame* pKF);

    std::vector<IMUData> GetVectorIMUData(void);
    void AppendIMUDataToFront(KeyFrame* pPrevKF);

    /// 计算关键帧的预计分
    void ComputePreInt(void);

    const IMUPreintegrator & GetIMUPreInt(void);

    /// use the computed camera pose to update the body pose
    void UpdateNavStatePVRFromTcw(const cv::Mat &Tcw,const cv::Mat &Tbc);

    /// use the imupreint result (body pose) to update the current keyframe pose
    void UpdatePoseFromNS(const cv::Mat &Tbc);

    /// 公式(33): 将imupreint积分得到的dR,dP,dV作用到NavState(IMU坐标系)
    void UpdateNavState(const IMUPreintegrator& IMUPreInt, const Vector3d& gw);

    void UpdateNavState(const IMUPreintegrator& IMUPreInt, const Vector3d& gw,
                        const Vector3d& dbiasg, const Vector3d& dbiasa);

    void SetNavState(const NavState& ns);
    const NavState& GetNavState(void);
    void SetNavStateVel(const Vector3d &vel);
    void SetNavStatePos(const Vector3d &pos);
    void SetNavStateRot(const Matrix3d &rot);
    void SetNavStateRot(const Sophus::SO3 &rot);
    void SetNavStateBiasGyr(const Vector3d &bg);
    void SetNavStateBiasAcc(const Vector3d &ba);
    void SetNavStateDeltaBg(const Vector3d &dbg);
    void SetNavStateDeltaBa(const Vector3d &dba);

    /// initial mNavState and Set bias as bias+delta_bias, and reset the delta_bias term
    void SetInitialNavStateAndBias(const NavState& ns);

    // Variables used by loop closing
    NavState mNavStateGBA;       //mTcwGBA
    NavState mNavStateBefGBA;    //mTcwBefGBA

    bool GetMonoVIEnable(void);
    void SetMonoVIEnable(bool flag=false);

protected:

    std::mutex mMutexPrevKF;
    std::mutex mMutexNextKF;
    KeyFrame* mpPrevKeyFrame;
    KeyFrame* mpNextKeyFrame;

    // P, V, R, bg, ba, delta_bg, delta_ba (delta_bx is for optimization update)
    std::mutex mMutexNavState;
    NavState mNavState;

    // IMU Data from lask KeyFrame to this KeyFrame
    std::mutex mMutexIMUData;
    std::vector<IMUData> mvIMUData;
    IMUPreintegrator mIMUPreInt;

private:
    bool mbMonoVIEnable;

/********************************************************************************/
/***************************** for VI-ORB_SLAM2 End *****************************/
/********************************************************************************/


public:
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool pairWeightComp(const pair<int,KeyFrame*> &a, const pair<int,KeyFrame*> &b)
    {
        return a.first>b.first; // 降序
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
