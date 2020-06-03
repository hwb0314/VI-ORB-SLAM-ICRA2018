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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include<mutex>

//#include "../src/IMU/imudata.h"


namespace ORB_SLAM2
{
long unsigned int KeyFrame::nNextId=0;

/// for VI-ORB_SLAM2
/********************************************************************************/
/**************************** for VI-ORB_SLAM2 Start ****************************/
/********************************************************************************/
void KeyFrame::SetIMUData(std::vector<IMUData> vIMUData)
{
    SetMonoVIEnable(true);
    unique_lock<mutex> lock(mMutexIMUData);
    mvIMUData = vIMUData;
}
void KeyFrame::SetPrevAndNextKeyFrame(KeyFrame* pPrevKF)
{
    unique_lock<mutex> lock(mMutexPrevKF);
    if(pPrevKF)
    {
        pPrevKF->SetNextKeyFrame(this);
    }
    mpPrevKeyFrame = pPrevKF;
    mpNextKeyFrame = NULL;
}

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB, std::vector<IMUData> vIMUData, KeyFrame* pPrevKF):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
{
    mbMonoVIEnable = true;

    mvIMUData = vIMUData;

    if(pPrevKF)
    {
        pPrevKF->SetNextKeyFrame(this);
    }
    mpPrevKeyFrame = pPrevKF;
    mpNextKeyFrame = NULL;

    //-------------------

    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);
}

KeyFrame* KeyFrame::GetPrevKeyFrame(void)
{
    unique_lock<mutex> lock(mMutexPrevKF);
    return mpPrevKeyFrame;
}

KeyFrame* KeyFrame::GetNextKeyFrame(void)
{
    unique_lock<mutex> lock(mMutexNextKF);
    return mpNextKeyFrame;
}

void KeyFrame::SetPrevKeyFrame(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexPrevKF);
    mpPrevKeyFrame = pKF;
}

void KeyFrame::SetNextKeyFrame(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexNextKF);
    mpNextKeyFrame = pKF;
}

std::vector<IMUData> KeyFrame::GetVectorIMUData(void)
{
    unique_lock<mutex> lock(mMutexIMUData);
    return mvIMUData;
}

void KeyFrame::AppendIMUDataToFront(KeyFrame* pPrevKF)
{
    std::vector<IMUData> vimunew = pPrevKF->GetVectorIMUData();
    {
        unique_lock<mutex> lock(mMutexIMUData);
        vimunew.insert(vimunew.end(), mvIMUData.begin(), mvIMUData.end());
        mvIMUData = vimunew;
    }
}

/**
 * @brief ComputePreInt
 *  计算关键帧的预计分
 * @param
 */
void KeyFrame::ComputePreInt(void){
    unique_lock<mutex> lock(mMutexIMUData);

    if(mvIMUData.empty())
    {
        if(0 != mnId){
            std::cerr << "mvIMUData is empty, this shouldn't happen when mnId != 0, mnId = " << mnId << std::endl;
        }
        return;
    }

    if(NULL == mpPrevKeyFrame)
    {
        if (0 != mnId){     // the mpPrevKeyFrame of the first KeyFrame is NULL
            std::cerr<<"previous KeyFrame is NULL, pre-integrator not changed. id: " << mnId << std::endl;
        }
        return;
    }
    else
    {
        // Debug log
        //cout<<std::fixed<<std::setprecision(3)<<
        //      "gyro bias: "<<mNavState.Get_BiasGyr().transpose()<<
        //      ", acc bias: "<<mNavState.Get_BiasAcc().transpose()<<endl;
        //cout<<std::fixed<<std::setprecision(3)<<
        //      "pre-int terms. prev KF time: "<<mpPrevKeyFrame->mTimeStamp<<endl<<
        //      "pre-int terms. this KF time: "<<mTimeStamp<<endl<<
        //      "imu terms times: "<<endl;

        // Reset pre-integrator first
        mIMUPreInt.reset();

        // IMU pre-integration integrates IMU data from last to current, but the bias is from lasr
        Vector3d bg = mpPrevKeyFrame->GetNavState().Get_BiasGyr();
        Vector3d ba = mpPrevKeyFrame->GetNavState().Get_BiasAcc();

        // remember to consider the gap between the mpPrevKeyFrame and the first IMU
        {
            const IMUData& imu = mvIMUData.front();
            double dt = imu._t - mpPrevKeyFrame->mTimeStamp;
            mIMUPreInt.update(imu._g-bg, imu._a-ba, dt);

            // Test log
            if(dt < 0)
            {
                cerr<<std::fixed<<std::setprecision(3)<<"dt = "<<dt<<", mpPrevKeyFrame vs the first imu time: "<<mpPrevKeyFrame->mTimeStamp<<" vs "<<imu._t<<endl;
                cerr<<std::fixed<<std::setprecision(3)<<"Suggestion: check the time delay setting between image and imu, may be it is too large." << endl;
                std::cerr.unsetf ( std::ios::showbase );                // deactivate showbase
            }
        }

        // integrate each imu
        for(size_t i=0, iend=mvIMUData.size(); i<iend; i++)
        {
            const IMUData& imu = mvIMUData[i];
            double nextt;
            if(i==iend-1)
                nextt = mTimeStamp;         // last IMU, next is this Keyframe
            else
                nextt = mvIMUData[i+1]._t;  // regular condition, next is imu data


            // delta time
            double dt = nextt - imu._t;
            // update the pre-integrator
            mIMUPreInt.update(imu._g-bg, imu._a-ba, dt);

            // Test log
            if(dt < 0)
            {
                cerr<<std::fixed<<std::setprecision(3)<<"dt = "<<dt<<", this vs next imu time: "<<imu._t<<" vs "<<nextt<<endl;
                cerr<<std::fixed<<std::setprecision(3)<<"Suggestion: check the time delay setting between image and imu, may be it is too large." << endl;
                std::cerr.unsetf ( std::ios::showbase );                // deactivate showbase
            }
        }
    }
}

const IMUPreintegrator& KeyFrame::GetIMUPreInt(void)
{
    unique_lock<mutex> lock(mMutexIMUData);
    return mIMUPreInt;
}

/**
 * @brief KeyFrame::UpdateNavStatePVRFromTcw
 * @param Tcw
 * @param Tbc
 * use the computed camera pose to update the body pose
 */
void KeyFrame::UpdateNavStatePVRFromTcw(const cv::Mat &Tcw,const cv::Mat &Tbc)
{
    unique_lock<mutex> lock(mMutexNavState);
    cv::Mat Twb = Converter::toCvMatInverse(Tbc*Tcw);
    Matrix3d Rwb = Converter::toMatrix3d(Twb.rowRange(0,3).colRange(0,3));
    Vector3d Pwb = Converter::toVector3d(Twb.rowRange(0,3).col(3));

    Matrix3d Rw1 = mNavState.Get_RotMatrix();
    Vector3d Vw1 = mNavState.Get_V();
    Vector3d Vw2 = Rwb*Rw1.transpose()*Vw1;     // bV1 = bV2 ==> Rwb1^T*wV1 = Rwb2^T*wV2 ==> wV2 = Rwb2*Rwb1^T*wV1

    mNavState.Set_Pos(Pwb);
    mNavState.Set_Vel(Vw2);
    mNavState.Set_Rot(Rwb);
}

/**
 * @brief KeyFrame::UpdatePoseFromNS
 * @param Tbc
 * use the imupreint result (body pose) to update the current keyframe pose
 */
void KeyFrame::UpdatePoseFromNS(const cv::Mat &Tbc)
{
    cv::Mat Rbc = Tbc.rowRange(0,3).colRange(0,3).clone();      // missing .clone()
    cv::Mat Pbc = Tbc.rowRange(0,3).col(3).clone();             // missing .clone()

    cv::Mat Rwb = Converter::toCvMat(mNavState.Get_RotMatrix());
    cv::Mat Pwb = Converter::toCvMat(mNavState.Get_P());

    cv::Mat Rcw = (Rwb*Rbc).t();
    cv::Mat Pwc = Rwb*Pbc + Pwb;
    cv::Mat Pcw = -Rcw*Pwc;

    cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
    Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
    Pcw.copyTo(Tcw.rowRange(0,3).col(3));

    SetPose(Tcw);
}

/**
 * @brief 将imupreint积分得到的dR,dP,dV作用到NavState(IMU坐标系)
 * 公式(33)
 */
void KeyFrame::UpdateNavState(const IMUPreintegrator& IMUPreInt, const Vector3d& gw){
    unique_lock<mutex> lock(mMutexNavState);
    Converter::updateNS(mNavState, IMUPreInt, gw);
}

void KeyFrame::UpdateNavState(const IMUPreintegrator& IMUPreInt, const Vector3d& gw,
                              const Vector3d& dbiasg, const Vector3d& dbiasa){
    unique_lock<mutex> lock(mMutexNavState);
    Converter::updateNS(mNavState, IMUPreInt, gw, dbiasg, dbiasa);
}

void KeyFrame::SetNavState(const NavState& ns){
    unique_lock<mutex> lock(mMutexNavState);
    mNavState = ns;
}

const NavState& KeyFrame::GetNavState(void){
    unique_lock<mutex> lock(mMutexNavState);
    return mNavState;
}

void KeyFrame::SetNavStateVel(const Vector3d &vel){
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_Vel(vel);
}

void KeyFrame::SetNavStatePos(const Vector3d &pos){
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_Pos(pos);
}

void KeyFrame::SetNavStateRot(const Matrix3d &rot){
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_Rot(rot);
}

void KeyFrame::SetNavStateRot(const Sophus::SO3 &rot){
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_Rot(rot);
}

void KeyFrame::SetNavStateBiasGyr(const Vector3d &bg){
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_BiasGyr(bg);     //// wrong with: mNavState.Set_DeltaBiasGyr(bg);
}

void KeyFrame::SetNavStateBiasAcc(const Vector3d &ba){
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_BiasAcc(ba);
}

void KeyFrame::SetNavStateDeltaBg(const Vector3d &dbg){
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_DeltaBiasGyr(dbg);
}

void KeyFrame::SetNavStateDeltaBa(const Vector3d &dba){
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_DeltaBiasAcc(dba);
}

void KeyFrame::SetInitialNavStateAndBias(const NavState& ns){
    unique_lock<mutex> lock(mMutexNavState);
    mNavState = ns;
    // set bias as bias+delta_bias, and reset the delta_bias term
    mNavState.Set_BiasGyr(ns.Get_BiasGyr()+ns.Get_dBias_Gyr());
    mNavState.Set_BiasAcc(ns.Get_BiasAcc()+ns.Get_dBias_Acc());
    mNavState.Set_DeltaBiasGyr(Vector3d::Zero());
    mNavState.Set_DeltaBiasAcc(Vector3d::Zero());
}

bool KeyFrame::GetMonoVIEnable(void)
{
    return mbMonoVIEnable;
}

void KeyFrame::SetMonoVIEnable(bool flag)
{
    mbMonoVIEnable = flag;
}

/********************************************************************************/
/***************************** for VI-ORB_SLAM2 End *****************************/
/********************************************************************************/




KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
{
    mbMonoVIEnable = false;

    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);    
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

//--------------------------------------------

//void KeyFrame::ComputeBoW()
//{
//    if(mBowVec.empty() || mFeatVec.empty())
//    {
////        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
////        // Feature vector associate features with nodes in the 4th level (from leaves up)
////        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
////        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);

//        //--------------------------------------------
//        DBoW2::BowVector mBowVec_2_0;
//        DBoW2::FeatureVector mFeatVec_2_0;
//        DBoW2::BowVector mBowVec_2_1;
//        DBoW2::FeatureVector mFeatVec_2_1;
//        vector<cv::Mat> vCurrentDesc_2_0 = Converter::toDescriptorVector(mDescriptors,2,0);
//        vector<cv::Mat> vCurrentDesc_2_1 = Converter::toDescriptorVector(mDescriptors,2,1);

//        mpORBvocabulary->transform(vCurrentDesc_2_0,mBowVec_2_0,mFeatVec_2_0,4);
//        mpORBvocabulary->transform(vCurrentDesc_2_1,mBowVec_2_1,mFeatVec_2_1,4);


////        std::cout << "mBowVec.size()="<<mBowVec.size() <<", mBowVec_2_0.size()="<<mBowVec_2_0.size()<<", mBowVec_2_1.size()="<<mBowVec_2_1.size()<<endl;
////        std::cout << "mFeatVec.size()="<<mFeatVec.size() <<", mFeatVec_2_0.size()="<<mFeatVec_2_0.size()<<", mFeatVec_2_1.size()="<<mFeatVec_2_1.size()<<endl;
////        if(mBowVec.size() == ( mBowVec_2_0.size()+ mBowVec_2_1.size() ))
////            std::cout << "yyyyy1" << std::endl;
////        if(mFeatVec.size() == ( mFeatVec_2_0.size()+ mFeatVec_2_1.size() ))
////            std::cout << "yyyyy2" << std::endl;

////        int count=0;
////        for(auto it=mBowVec_2_0.begin(),itend=mBowVec_2_0.end();it!=itend;it++)
////        {
////            if( mBowVec_2_1.find(it->first) != mBowVec_2_1.end())
////                count++;
////        }
////        std::cout << "count = " << count << std::endl;

//        // merge BowVector
////        DBoW2::BowVector tmpBowVec;
//        for(auto it=mBowVec_2_0.begin(),itend=mBowVec_2_0.end();it!=itend;it++)
//        {
//            mBowVec.addWeight(it->first, it->second);
//        }
//        for(auto it=mBowVec_2_1.begin(),itend=mBowVec_2_1.end();it!=itend;it++)
//        {
//            mBowVec.addWeight(it->first, it->second);
//        }

////        std::cout << "mBowVec.size()="<<mBowVec.size() <<", tmpBowVec.size()="<<tmpBowVec.size()<<endl;

//        // merge FeatureVector
////        DBoW2::FeatureVector mFeatVec;
//        for(auto it=mFeatVec_2_0.begin(),itend=mFeatVec_2_0.end();it!=itend;it++)
//        {
//            for(auto i=it->second.begin(),iend=it->second.end();i!=iend;i++)
//                mFeatVec.addFeature(it->first, *i);
//        }
//        for(auto it=mFeatVec_2_1.begin(),itend=mFeatVec_2_1.end();it!=itend;it++)
//        {
//            for(auto i=it->second.begin(),iend=it->second.end();i!=iend;i++)
//                mFeatVec.addFeature(it->first, *i);
//        }
////        std::cout << "mFeatVec.size()="<<mFeatVec.size() <<", tmpFeatVec.size()="<<tmpFeatVec.size()<<endl;

//        //--------------------------------------------
//    }
//}

//--------------------------------------------


void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center;
}

cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}


cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

// /// 我的改动1 --- 添加, 应注释掉上面相同的函数
// void KeyFrame::UpdateBestCovisibles()
// {
//     unique_lock<mutex> lock(mMutexConnections);
//     vector<pair<int,KeyFrame*> > vPairs;
//     vPairs.reserve(mConnectedKeyFrameWeights.size());
//     // 取出所有连接的关键帧，mConnectedKeyFrameWeights的类型为std::map<KeyFrame*,int>，而vPairs变量将权重(共视的3D点数)放在前面，利于排序
//     for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
//        vPairs.push_back(make_pair(mit->second,mit->first));

//     // 按权重进行排序. sort, 指定从大到小
//     sort(vPairs.begin(),vPairs.end(), KeyFrame::pairWeightComp);

//     std::vector<KeyFrame*> vKFs; vKFs.reserve(vPairs.size());
//     std::vector<int> vWs;  vWs.reserve(vPairs.size());
//     for(size_t i=0, iend=vPairs.size(); i<iend;i++)
//     {
//         vKFs.push_back(vPairs[i].second);
//         vWs.push_back(vPairs[i].first);
//     }

//     mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(vKFs.begin(),vKFs.end());
//     mvOrderedWeights = vector<int>(vWs.begin(), vWs.end());
// } // 我的改动1 结束

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    map<KeyFrame*,int>::iterator mit_begin = mConnectedKeyFrameWeights.begin();
    map<KeyFrame*,int>::iterator mit_end   = mConnectedKeyFrameWeights.end();
    // for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
    for(map<KeyFrame*,int>::iterator mit=mit_begin;mit!=mit_end;mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

/**
 * @brief 更新图的连接
 *
 * 1. 首先获得该关键帧的所有MapPoint点，统计观测到这些3d点的每个关键帧与其它所有关键帧之间的共视程度
 *    对每一个找到的关键帧，建立一条边，边的权重是该关键帧与当前关键帧公共3d点的个数。
 * 2. 并且该权重必须大于一个阈值，如果没有超过该阈值的权重，那么就只保留权重最大的边（与其它关键帧的共视程度比较高）
 * 3. 对这些连接按照权重从大到小进行排序，以方便将来的处理
 *    更新完covisibility图之后，如果没有初始化过，则初始化为连接权重最大的边（与其它关键帧共视程度最高的那个关键帧），类似于最大生成树
 */
void KeyFrame::UpdateConnections()
{
    // 在没有执行这个函数前，关键帧只和MapPoints之间有连接关系，这个函数可以更新关键帧之间的连接关系

    //===============1==================================
    map<KeyFrame*,int> KFcounter;       // 关键帧-权重，权重为其它关键帧与当前关键帧共视3d点的个数

    vector<MapPoint*> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe， check in which other keyframes are they seen
    //Increase counter for those keyframes
    // 通过3D点间接统计可以观测到这些3D点的所有关键帧之间的共视程度
    // 即统计每一个关键帧都有多少关键帧与它存在共视关系，统计结果放在KFcounter
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        // 对于每一个MapPoint点，observations记录了可以观测到该MapPoint的所有关键帧以及对应的索引
        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            // 除去自身,自己与自己不算共视
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //===============2==================================
    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;

    // vPairs记录与其它关键帧共视帧数大于th的关键帧
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        // if: 找到权重最大的关键帧(共视程度最高的关键帧)
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            // 对应权重需要大于阈值，对这些关键帧建立连接
            vPairs.push_back(make_pair(mit->second,mit->first));
            // 更新KFcounter中该关键帧的mConnectedKeyFrameWeights
            // 更新其它KeyFrame的mConnectedKeyFrameWeights，更新其它关键帧与当前帧的连接权重
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    // 如果没有超过阈值的权重，则对权重最大的关键帧建立连接
    if(vPairs.empty())
    {
        // 如果每个关键帧与当前关键帧共视的个数都少于th，
        // 那就只更新与当前关键帧共视程度最高的关键帧的mConnectedKeyFrameWeights
        // 这是对之前th这个阈值可能过高的一个补丁
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    // vPairs里存的都是相互共视程度比较高的关键帧和共视权重，由小到大
    // * 用list的push_front()效率较低. 可将vPair直接从大到小排列,而后用vector的push_back()
    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        // 因为vPairs按权重由小到大排列,因而得用list.push_front,使lKFs与lWs按权重由大到小排列
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    //===============3==================================
    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        // 更新图的连接(权重)
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        // 更新生成树的连接
        if(mbFirstConnection && mnId!=0)
        {
            // 初始化该关键帧的父关键帧为共视程度最高的那个关键帧
            mpParent = mvpOrderedConnectedKeyFrames.front();
            // 建立双向连接关系
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }
    }

//     /// 我的改动2 --- 添加, 应注释掉上面类似的代码
//     // 按权重进行排序. sort, 指定从大到小
//     sort(vPairs.begin(),vPairs.end(), KeyFrame::pairWeightComp);
//     std::vector<KeyFrame*> vKFs; vKFs.reserve(vPairs.size());
//     std::vector<int> vWs;  vWs.reserve(vPairs.size());
//     for(size_t i=0, iend=vPairs.size(); i<iend;i++)
//     {
//         vKFs.push_back(vPairs[i].second);
//         vWs.push_back(vPairs[i].first);
//     }

//     {
//         unique_lock<mutex> lockCon(mMutexConnections);

//         // mspConnectedKeyFrames = spConnectedKeyFrames;
//         mConnectedKeyFrameWeights = KFcounter;
//         mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(vKFs.begin(),vKFs.end());
//         mvOrderedWeights = vector<int>(vWs.begin(), vWs.end());

//         if(mbFirstConnection && mnId!=0)
//         {
//             mpParent = mvpOrderedConnectedKeyFrames.front();
//             mpParent->AddChild(this);
//             mbFirstConnection = false;
//         }
//     }   // 我的改动2 结束

}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

/**
 * @brief 重要函数
 *      1. 剔除关键帧,并删除共视关键帧与自己的联系,且更新共视关键帧的父节点与子节点
 *      2. 更新前后两个关键帧(pPrevKF与pNextKF)的关系,将当前帧的IMU测量值划分给pNextKF
 */
void KeyFrame::SetBadFlag()
{   
    // Test log
    if(mbBad)
    {
        vector<KeyFrame*> vKFinMap =mpMap->GetAllKeyFrames();
        std::set<KeyFrame*> KFinMap(vKFinMap.begin(),vKFinMap.end());
        if(KFinMap.count(this))
        {
            cerr<<"this bad KF is still in map?"<<endl;
            mpMap->EraseKeyFrame(this);
        }
        mpKeyFrameDB->erase(this);
        //cerr<<"KeyFrame "<<mnId<<" is already bad. Set bad return"<<endl;
        return;
    }

    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==0)
            return;
        else if(mbNotErase) // mbNotErase表示不应该擦除该KeyFrame，于是把mbToBeErased置为true，表示已经擦除了，其实没有擦除
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(this);  // 让其它的KeyFrame删除与自己的联系

    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this);    // 让与自己有联系的MapPoint删除与自己的联系

    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        //清空自己与其它关键帧之间的联系
        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        // 如果这个关键帧有自己的孩子关键帧，告诉这些子关键帧，它们的父关键帧不行了，赶紧找新的父关键帧
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            // 遍历每一个子关键帧，让它们更新它们指向的父关键帧
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                // 子关键帧遍历每一个与它相连的关键帧（共视关键帧）
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        // 如果该帧的子节点和父节点（祖孙节点）之间存在连接关系（共视）
                        // 举例：B-->A（B的父节点是A） C-->B（C的父节点是B） D--C（D与C相连） E--C（E与C相连） F--C（F与C相连） D-->A（D的父节点是A） E-->A（E的父节点是A）
                        //      现在B挂了，于是C在与自己相连的D、E、F节点中找到父节点指向A的D
                        //      此过程就是为了找到可以替换B的那个节点。
                        // 上面例子中，B为当前要设置为SetBadFlag的关键帧
                        //           A为spcit，也即sParentCandidates
                        //           C为pKF,pC，也即mspChildrens中的一个
                        //           D、E、F为vpConnected中的变量，由于C与D间的权重 比 C与E间的权重大，因此D为pP

                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(this);
        mTcp = Tcw*mpParent->GetPoseInverse();
        mbBad = true;
    }

    /// For VI-ORB_SLAM
    // 若删除当前关键帧,需要更新前后两个关键帧(pPrevKF与pNextKF)的关系,将当前帧的IMU测量值划分给pNextKF
    // 原本pPrevKF的pNextKF是当前关键帧,现在当前关键帧挂了,要为pPrevKF重新指定pNextKF.
    // 原本pNextKF的pPrevKF是当前关键帧,现在当前关键帧挂了,要为pNextKF重新指定pPrevKF.
    // Update Prev/Next KeyFrame for prev/next
    if(mbMonoVIEnable)
    {
//        cout << "KeyFrame::SetBadFlag() for VI-ORB_SLAM, Begin. Current id: "<< mnId << endl;
        KeyFrame* pPrevKF = GetPrevKeyFrame();
        KeyFrame* pNextKF = GetNextKeyFrame();
//        cout << "**** 1" << endl;
        if(pPrevKF)
            pPrevKF->SetNextKeyFrame(pNextKF);
        if(pNextKF)
            pNextKF->SetPrevKeyFrame(pPrevKF);
//        cout << "**** 2" << endl;
        SetPrevKeyFrame(NULL);
        SetNextKeyFrame(NULL);
//        cout << "**** 3" << endl;
        // TODO: this happend once. Log: Current id = 1.
        // Test log.
        if(!pPrevKF) cerr<<"It's culling the first KF? pPrevKF=NULL. Current id: "<<mnId<<endl;
        if(!pNextKF) cerr<<"It's culling the latest KF? pNextKF=NULL. Current id: "<<mnId<<endl;
        // TODO
//        cout << "**** 4" << endl;
        if(pPrevKF && pNextKF)
        {
            if(pPrevKF->isBad()) cerr<<"Prev KF isbad in setbad. previd: "<<pPrevKF->mnId<<", current id"<<mnId<<endl;
            if(pNextKF->isBad()) cerr<<"Next KF isbad in setbad. previd: "<<pNextKF->mnId<<", current id"<<mnId<<endl;

            //Debug log, compare the bias of culled KF and the replaced one
            //cout<<"culled KF bg/ba: "<<mNavState.Get_BiasGyr().transpose()<<", "<<mNavState.Get_BiasAcc().transpose()<<endl;
            //cout<<"next KF bg/ba: "<<pNextKF->GetNavState().Get_BiasGyr().transpose()<<", "<<pNextKF->GetNavState().Get_BiasAcc().transpose()<<endl;

            // Update IMUData for NextKF
            pNextKF->AppendIMUDataToFront(this);    // 将当前关键帧的IMU数据赋给pNextKF
            // Re-compute pre-integrator
            pNextKF->ComputePreInt();       // 重新计算pNextKF的预积分
        }
//        cout << "KeyFrame::SetBadFlag() for VI-ORB_SLAM, End" << endl;
    }

    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

} //namespace ORB_SLAM
