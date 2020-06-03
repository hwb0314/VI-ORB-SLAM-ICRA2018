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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>

#include "../src/IMU/NavState.h"    /// for VI-ORB_SLAM2
#include "../src/IMU/configparam.h"
#include "../src/IMU/NavState.h"

using namespace std;

namespace ORB_SLAM2
{
/********************************************************************************/
/**************************** for VI-ORB_SLAM2 Start ****************************/
/********************************************************************************/
bool Tracking::SetConfigParam(ConfigParam* pParams)
{
    mpParams = pParams;
    return true;
}

cv::Mat Tracking::GrabImageMonoVI(const cv::Mat &im, const std::vector<IMUData> &vimu, const double &timestamp)
{
    Timer frame_processing_timer;

    // Step 1: append new IMU data
    mvIMUSinceLastKF.insert(mvIMUSinceLastKF.end(), vimu.begin(), vimu.end());
    mImGray = im;

    // Step 2: change into gray image
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    // Step 3: construct Frame, it cost about more than 20ms
    Timer frame_construct_timer;
    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp, vimu, mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp, vimu, mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    mTimeOfConstructFrame = frame_construct_timer.runTime_ms();   // record time
    if(mpParams->GetDispalyTimeStatistic()) std::cout << "[INFO] -->> Construct Frame time is: " << mTimeOfConstructFrame << " ms" << std::endl;

//    ///////////////////////////////////////
//    if(!mpLocalMapper->AcceptKeyFrames())
//        std::cout << BLUE"[INFO] 1--mpLocalMapper->AcceptKeyFrames() = false" << ", mCurrentFrame.mnId = " << mCurrentFrame.mnId << RESET << std::endl;
//    else
//        std::cout << GREEN"[INFO] 1--mpLocalMapper->AcceptKeyFrames() = true" << ", mCurrentFrame.mnId = " << mCurrentFrame.mnId << RESET << std::endl;

//    ///////////////////////////////////////

    // Step 4: track VIO
    Timer track_timer;
    Track();

    mTimeOfTrack = track_timer.runTime_ms();    // record time
    if(mpParams->GetDispalyTimeStatistic()) std::cout << "[INFO] <<-- Tracking time is: " << mTimeOfTrack << " ms" << std::endl;

    mTimeOfProcessingFrame = frame_processing_timer.runTime_ms();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::RecomputeIMUBiasAndCurrentNavstate(NavState& nscur)
{
    size_t N = mv20FramesReloc.size();

    //Test log
    if(N!=20) cerr<<"Frame vector size not 20 to compute bias after reloc??? size: "<<mv20FramesReloc.size()<<endl;

    // Estimate gyr bias
    Vector3d bg = Optimizer::OptimizeInitialGyroBias(mv20FramesReloc);
    // Update gyr bias of Frames
    for(size_t i=0; i<N; i++)
    {
        Frame& frame = mv20FramesReloc[i];
        //Test log
        if(frame.GetNavState().Get_BiasGyr().norm()!=0 || frame.GetNavState().Get_dBias_Gyr().norm()!=0)
            cerr<<"Frame "<<frame.mnId<<" gyr bias or delta bias not zero???"<<endl;

        frame.SetNavStateBiasGyr(bg);
    }
    // Re-compute IMU pre-integration
    vector<IMUPreintegrator> v19IMUPreint;
    v19IMUPreint.reserve(20-1);
    for(size_t i=0; i<N; i++)
    {
        if(i==0)
            continue;

        const Frame& Fi = mv20FramesReloc[i-1];
        const Frame& Fj = mv20FramesReloc[i];

        IMUPreintegrator imupreint;
        Fj.ComputeIMUPreIntSinceLastFrame(&Fi,imupreint);
        v19IMUPreint.push_back(imupreint);
    }
    // Construct [A1;A2;...;AN] * ba = [B1;B2;.../BN], solve ba
    cv::Mat A = cv::Mat::zeros(3*(N-2),3,CV_32F);
    cv::Mat B = cv::Mat::zeros(3*(N-2),1,CV_32F);
    const cv::Mat& gw = mpLocalMapper->GetGravityVec();
    const cv::Mat& Tcb = mpParams->GetMatT_cb();
    for(size_t i=0; i<N-2; i++)
    {
        const Frame& F1 = mv20FramesReloc[i];
        const Frame& F2 = mv20FramesReloc[i+1];
        const Frame& F3 = mv20FramesReloc[i+2];
        const IMUPreintegrator& PreInt12 = v19IMUPreint[i];
        const IMUPreintegrator& PreInt23 = v19IMUPreint[i+1];
        // Delta time between frames
        double dt12 = PreInt12.getDeltaTime();
        double dt23 = PreInt23.getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(PreInt12.getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(PreInt12.getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(PreInt23.getDeltaP());
        cv::Mat Jpba12 = Converter::toCvMat(PreInt12.getJPBiasa());
        cv::Mat Jvba12 = Converter::toCvMat(PreInt12.getJVBiasa());
        cv::Mat Jpba23 = Converter::toCvMat(PreInt23.getJPBiasa());
        // Pose of body in world frame
        cv::Mat Twb1 = Converter::toCvMatInverse(F1.mTcw)*Tcb;
        cv::Mat Twb2 = Converter::toCvMatInverse(F2.mTcw)*Tcb;
        cv::Mat Twb3 = Converter::toCvMatInverse(F3.mTcw)*Tcb;
        // Position of body, Pwb
        cv::Mat pb1 = Twb1.rowRange(0,3).col(3);
        cv::Mat pb2 = Twb2.rowRange(0,3).col(3);
        cv::Mat pb3 = Twb3.rowRange(0,3).col(3);
        // Rotation of body, Rwb
        cv::Mat Rb1 = Twb1.rowRange(0,3).colRange(0,3);
        cv::Mat Rb2 = Twb2.rowRange(0,3).colRange(0,3);
        //cv::Mat Rb3 = Twb3.rowRange(0,3).colRange(0,3);
        // Stack to A/B matrix
        // Ai * ba = Bi
        cv::Mat Ai = Rb1*Jpba12*dt23 - Rb2*Jpba23*dt12 - Rb1*Jvba12*dt12*dt23;
        cv::Mat Bi = (pb2-pb3)*dt12 + (pb2-pb1)*dt23 + Rb2*dp23*dt12 - Rb1*dp12*dt23 + Rb1*dv12*dt12*dt23 + 0.5*gw*(dt12*dt12*dt23+dt12*dt23*dt23);
        Ai.copyTo(A.rowRange(3*i+0,3*i+3));
        Bi.copyTo(B.rowRange(3*i+0,3*i+3));

        //Test log
        if(fabs(F2.mTimeStamp-F1.mTimeStamp-dt12)>1e-6 || fabs(F3.mTimeStamp-F2.mTimeStamp-dt23)>1e-6) cerr<<"delta time not right."<<endl;

        //        // lambda*s + phi*dthetaxy + zeta*ba = psi
        //        cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
        //        cv::Mat phi = - 0.5*(dt12*dt12*dt23 + dt12*dt23*dt23)*Rwi*SkewSymmetricMatrix(GI);  // note: this has a '-', different to paper
        //        cv::Mat zeta = Rc2*Rcb*Jpba23*dt12 + Rc1*Rcb*Jvba12*dt12*dt23 - Rc1*Rcb*Jpba12*dt23;
        //        cv::Mat psi = (Rc1-Rc2)*pcb*dt23 + Rc1*Rcb*dp12*dt23 - (Rc2-Rc3)*pcb*dt12
        //                     - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt23*dt12 - 0.5*Rwi*GI*(dt12*dt12*dt23 + dt12*dt23*dt23); // note:  - paper
        //        lambda.copyTo(C.rowRange(3*i+0,3*i+3).col(0));
        //        phi.colRange(0,2).copyTo(C.rowRange(3*i+0,3*i+3).colRange(1,3)); //only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
        //        zeta.copyTo(C.rowRange(3*i+0,3*i+3).colRange(3,6));
        //        psi.copyTo(D.rowRange(3*i+0,3*i+3));

    }

    // Use svd to compute A*x=B, x=ba 3x1 vector
    // A = u*w*vt, u*w*vt*x=B
    // Then x = vt'*winv*u'*B
    cv::Mat w2,u2,vt2;
    // Note w2 is 3x1 vector by SVDecomp()
    // A is changed in SVDecomp() with cv::SVD::MODIFY_A for speed
    cv::SVDecomp(A,w2,u2,vt2,cv::SVD::MODIFY_A);
    // Compute winv
    cv::Mat w2inv=cv::Mat::eye(3,3,CV_32F);
    for(int i=0;i<3;i++)
    {
        if(fabs(w2.at<float>(i))<1e-10)
        {
            w2.at<float>(i) += 1e-10;
            // Test log
            cerr<<"w2(i) < 1e-10, w="<<endl<<w2<<endl;
        }
        w2inv.at<float>(i,i) = 1./w2.at<float>(i);
    }
    // Then y = vt'*winv*u'*B
    cv::Mat ba_cv = vt2.t()*w2inv*u2.t()*B;
    Vector3d ba = Converter::toVector3d(ba_cv);

    // Update acc bias
    for(size_t i=0; i<N; i++)
    {
        Frame& frame = mv20FramesReloc[i];
        //Test log
        if(frame.GetNavState().Get_BiasAcc().norm()!=0 || frame.GetNavState().Get_dBias_Gyr().norm()!=0 || frame.GetNavState().Get_dBias_Acc().norm()!=0)
            cerr<<"Frame "<<frame.mnId<<" acc bias or delta bias not zero???"<<endl;

        frame.SetNavStateBiasAcc(ba);
    }

    // Compute Velocity of the last 2 Frames
    Vector3d Pcur;
    Vector3d Vcur;
    Matrix3d Rcur;
    {
        Frame& F1 = mv20FramesReloc[N-2];
        Frame& F2 = mv20FramesReloc[N-1];
        const IMUPreintegrator& imupreint = v19IMUPreint.back();
        const double dt12 = imupreint.getDeltaTime();
        const Vector3d dp12 = imupreint.getDeltaP();
        const Vector3d gweig = Converter::toVector3d(gw);
        const Matrix3d Jpba12 = imupreint.getJPBiasa();
        const Vector3d dv12 = imupreint.getDeltaV();
        const Matrix3d Jvba12 = imupreint.getJVBiasa();

        // Velocity of Previous Frame
        // P2 = P1 + V1*dt12 + 0.5*gw*dt12*dt12 + R1*(dP12 + Jpba*ba + Jpbg*0)
        cv::Mat Twb1 = Converter::toCvMatInverse(F1.mTcw)*Tcb;
        cv::Mat Twb2 = Converter::toCvMatInverse(F2.mTcw)*Tcb;
        Vector3d P1 = Converter::toVector3d(Twb1.rowRange(0,3).col(3));
        /*Vector3d */Pcur = Converter::toVector3d(Twb2.rowRange(0,3).col(3));
        Matrix3d R1 = Converter::toMatrix3d(Twb1.rowRange(0,3).colRange(0,3));
        /*Matrix3d */Rcur = Converter::toMatrix3d(Twb2.rowRange(0,3).colRange(0,3));
        Vector3d V1 = 1./dt12*( Pcur - P1 - 0.5*gweig*dt12*dt12 - R1*(dp12 + Jpba12*ba) );

        // Velocity of Current Frame
        Vcur = V1 + gweig*dt12 + R1*( dv12 + Jvba12*ba );

        // Test log
        if(F2.mnId != mCurrentFrame.mnId) cerr<<"framecur.mnId != mCurrentFrame.mnId. why??"<<endl;
        if(fabs(F2.mTimeStamp-F1.mTimeStamp-dt12)>1e-6) cerr<<"timestamp not right?? in compute vel"<<endl;
    }

    // Set NavState of Current Frame, P/V/R/bg/ba/dbg/dba
    nscur.Set_Pos(Pcur);
    nscur.Set_Vel(Vcur);
    nscur.Set_Rot(Rcur);
    nscur.Set_BiasGyr(bg);
    nscur.Set_BiasAcc(ba);
    nscur.Set_DeltaBiasGyr(Vector3d::Zero());
    nscur.Set_DeltaBiasAcc(Vector3d::Zero());

    //mv20FramesReloc
}

/// Wangjing
//void Tracking::PredictNavStateByIMU( bool bMapUpdated )
//{
//    if(!mpLocalMapper->GetVINSInited()) cerr<<"mpLocalMapper->GetVINSInited() not, shouldn't in PredictNavStateByIMU"<<endl;

//    // Map updated, optimize with last KeyFrame
//    if(mpLocalMapper->GetFirstVINSInited() || bMapUpdated)
//    {
////        std::cout << "[INFO] Map updated, optimize with last KeyFrame" << std::endl;
//        if(mpLocalMapper->GetFirstVINSInited() && !bMapUpdated) cerr<<RED"2-FirstVinsInit, but not bMapUpdated. shouldn't"<<RESET<<endl;

//        // Compute IMU Pre-integration
//        mIMUPreIntInTrack = GetIMUPreIntSinceLastKF(&mCurrentFrame, mpLastKeyFrame, mvIMUSinceLastKF);

//        // Get initial NavState&pose from Last KeyFrame
//        mCurrentFrame.SetInitialNavStateAndBias(mpLastKeyFrame->GetNavState());
//        mCurrentFrame.UpdateNavState(mIMUPreIntInTrack,Converter::toVector3d(mpLocalMapper->GetGravityVec()));
//        mCurrentFrame.UpdatePoseFromNS(mpParams->GetMatTbc());

////        // Test log. Display the poses of LastKF and mCurrentFrame
////        {
////            cv::Mat Rcw_lk = mpLastKeyFrame->GetPose().rowRange(0,3).colRange(0,3);
////            Eigen::Vector3d euler_lk = Converter::toEulerAngles(Rcw_lk);
////            std::cout << YELLOW"[INFO] 2---Last KeyFrame euler angle = ["
////                      << euler_lk[0] << ", "
////                      << euler_lk[1] << ", "
////                      << euler_lk[2] << "]" << ",  mpLastKeyFrame->mnFrameId = " << mpLastKeyFrame->mnFrameId << RESET << std::endl;
////            //-------------------------------
////            cv::Mat Rcw_lf = mLastFrame.GetRotationInverse().t();
////            Eigen::Vector3d euler_lf = Converter::toEulerAngles(Rcw_lf);
////            std::cout << YELLOW"[INFO] 3---Last frame pose = ["
////                      << euler_lf[0] << ", "
////                      << euler_lf[1] << ", "
////                      << euler_lf[2] << "]" << RESET << std::endl;
////            //-------------------------------
////            cv::Mat Rcw = mCurrentFrame.GetRotationInverse().t();
////            Eigen::Vector3d euler = Converter::toEulerAngles(Rcw);
////            std::cout << YELLOW"[INFO] 4---After Pre-integration since Last KeyFrame, current frame euler angle = ["
////                      << euler[0] << ", "
////                      << euler[1] << ", "
////                      << euler[2] << "]" <<  ", mvIMUSinceLastKF.size() = " <<  mvIMUSinceLastKF.size() << RESET << std::endl;
////        }

//        // Test log
//        // Updated KF by Local Mapping. Should be the same as mpLastKeyFrame
//        if(mCurrentFrame.GetNavState().Get_dBias_Acc().norm() > 1e-6) cerr<<"PredictNavStateByIMU1 current Frame dBias acc not zero"<<endl;
//        if(mCurrentFrame.GetNavState().Get_dBias_Gyr().norm() > 1e-6) cerr<<"PredictNavStateByIMU1 current Frame dBias gyr not zero"<<endl;
//    }
//    // Map not updated, optimize with last Frame
//    else
//    {
////        std::cout << "Map not updated, optimize with last Frame" << std::endl;

//        // Compute IMU Pre-integration
//        mIMUPreIntInTrack = GetIMUPreIntSinceLastFrame(&mCurrentFrame, &mLastFrame);

//        // Get initial pose from Last Frame
//        mCurrentFrame.SetInitialNavStateAndBias(mLastFrame.GetNavState());
//        mCurrentFrame.UpdateNavState(mIMUPreIntInTrack,Converter::toVector3d(mpLocalMapper->GetGravityVec()));
//        mCurrentFrame.UpdatePoseFromNS(mpParams->GetMatTbc());

//        // Test log
//        if(mCurrentFrame.GetNavState().Get_dBias_Acc().norm() > 1e-6) cerr<<"PredictNavStateByIMU2 current Frame dBias acc not zero"<<endl;
//        if(mCurrentFrame.GetNavState().Get_dBias_Gyr().norm() > 1e-6) cerr<<"PredictNavStateByIMU2 current Frame dBias gyr not zero"<<endl;
//    }
//}

/// Modified by Huangweibo
void Tracking::PredictNavStateByIMU( bool bMapUpdated )
{
    if(!mpLocalMapper->GetVINSInited()) cerr<<"mpLocalMapper->GetVINSInited() not, shouldn't in PredictNavStateByIMU"<<endl;

    // Map updated, optimize with last KeyFrame
//    if(mpLocalMapper->GetFirstVINSInited())
    if (bMapUpdated)
    {
        // Compute IMU Pre-integration
        mIMUPreIntInTrack = GetIMUPreIntSinceLastKF(&mCurrentFrame, mpLastKeyFrame, mvIMUSinceLastKF); // do not consider the bias update

        // Get initial NavState&pose from Last KeyFrame
        mCurrentFrame.SetInitialNavStateAndBias(mpLastKeyFrame->GetNavState());
        mCurrentFrame.UpdateNavState(mIMUPreIntInTrack,Converter::toVector3d(mpLocalMapper->GetGravityVec()));
//        mCurrentFrame.UpdateNavState(mIMUPreIntInTrack,Converter::toVector3d(mpLocalMapper->GetGravityVec()),
//                                     mpLastKeyFrame->GetNavState().Get_dBias_Gyr(), mpLastKeyFrame->GetNavState().Get_dBias_Acc());
        if(mpParams->GetEstimateExtrinsicBetweenCameraAndIMU())
            mCurrentFrame.UpdatePoseFromNS(mpLocalMapper->GetVINSInitTbc());
        else
            mCurrentFrame.UpdatePoseFromNS(mpParams->GetMatTbc());

        // Test log
        // Updated KF by Local Mapping. Should be the same as mpLastKeyFrame
        if(mCurrentFrame.GetNavState().Get_dBias_Acc().norm() > 1e-6) cerr<<"PredictNavStateByIMU1 current Frame dBias acc not zero"<<endl;
        if(mCurrentFrame.GetNavState().Get_dBias_Gyr().norm() > 1e-6) cerr<<"PredictNavStateByIMU1 current Frame dBias gyr not zero"<<endl;
    }
    else
    {
        // Compute IMU Pre-integration
        mIMUPreIntInTrack = GetIMUPreIntSinceLastFrame(&mCurrentFrame, &mLastFrame);

        // Get initial pose from Last Frame
        mCurrentFrame.SetInitialNavStateAndBias(mLastFrame.GetNavState());
        mCurrentFrame.UpdateNavState(mIMUPreIntInTrack,Converter::toVector3d(mpLocalMapper->GetGravityVec()));
//        mCurrentFrame.UpdateNavState(mIMUPreIntInTrack,Converter::toVector3d(mpLocalMapper->GetGravityVec()),
//                                     mLastFrame.GetNavState().Get_dBias_Gyr(), mLastFrame.GetNavState().Get_dBias_Acc());
        if(mpParams->GetEstimateExtrinsicBetweenCameraAndIMU())
            mCurrentFrame.UpdatePoseFromNS(mpLocalMapper->GetVINSInitTbc());
        else
            mCurrentFrame.UpdatePoseFromNS(mpParams->GetMatTbc());

        // Test log
        if(mCurrentFrame.GetNavState().Get_dBias_Acc().norm() > 1e-6) cerr<<"PredictNavStateByIMU2 current Frame dBias acc not zero"<<endl;
        if(mCurrentFrame.GetNavState().Get_dBias_Gyr().norm() > 1e-6) cerr<<"PredictNavStateByIMU2 current Frame dBias gyr not zero"<<endl;
    }
}

// Update last frame pose and NavState according to its reference keyframe
void Tracking::UpdateLastFrameWithIMU()
{
    // Update pose and NavState according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    // use the update camera pose to update the body pose
    if(mpParams->GetEstimateExtrinsicBetweenCameraAndIMU())
        mLastFrame.UpdateNavStatePVRFromTcw( mLastFrame.mTcw, mpLocalMapper->GetVINSInitTbc() );
    else
        mLastFrame.UpdateNavStatePVRFromTcw( mLastFrame.mTcw, mpParams->GetMatTbc() );
}

bool Tracking::TrackWithIMU(bool bMapUpdated)
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose and NavState according to its reference keyframe
    UpdateLastFrameWithIMU();

    // VINS has been inited in this function
    if(!mpLocalMapper->GetVINSInited()) cerr<<"local mapping VINS not inited. why call TrackWithIMU?"<<endl;

    // Predict NavState&Pose by IMU
    // And compute the IMU pre-integration for PoseOptimization
    PredictNavStateByIMU(bMapUpdated);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;

    int nmatches;

    // TODO: if bMapUpdated==true, then SearchByProjection with lastLastKeyFrame

    nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    // Test log
    {
        if(nmatches<20) std::cout << "[INFO] TrackWithIMU fail due to (nmatches<20)" << std::endl;
    }

    if(nmatches<20)
        return false;


    // Pose optimization. false: no need to compute marginalized for current Frame
//    if(mpLocalMapper->GetFirstVINSInited() ) //|| bMapUpdated)
    cv::Mat Tbc;
    if(mpParams->GetEstimateExtrinsicBetweenCameraAndIMU())     // estimate the Tbc
        Tbc = mpLocalMapper->GetVINSInitTbc();
    else
        Tbc = mpParams->GetMatTbc();

    if(bMapUpdated)
        Optimizer::PoseOptimization(&mCurrentFrame, mpLastKeyFrame, mIMUPreIntInTrack,
                                        mpLocalMapper->GetGravityVec(), Tbc, false);
    else
            Optimizer::PoseOptimization(&mCurrentFrame, &mLastFrame, mIMUPreIntInTrack,
                    mpLocalMapper->GetGravityVec(), Tbc, false);


    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    // Test log
    {
        if(nmatchesMap<10) std::cout << "[INFO] TrackWithIMU fail due to (nmatchesMap<10), nmatchesMap = " << nmatchesMap  << std::endl;
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackReferenceKeyFrameWithIMU()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
//    mCurrentFrame.SetPose(mLastFrame.mTcw);

    // Compute IMU Pre-integration
    mIMUPreIntInTrack = GetIMUPreIntSinceLastKF(&mCurrentFrame, mpLastKeyFrame, mvIMUSinceLastKF);

    // Get initial NavState&pose from Last KeyFrame
    mCurrentFrame.SetInitialNavStateAndBias(mpLastKeyFrame->GetNavState());
    mCurrentFrame.UpdateNavState(mIMUPreIntInTrack,Converter::toVector3d(mpLocalMapper->GetGravityVec()));
//    mCurrentFrame.UpdateNavState(mIMUPreIntInTrack,Converter::toVector3d(mpLocalMapper->GetGravityVec()),
//                                 mpLastKeyFrame->GetNavState().Get_dBias_Gyr(), mpLastKeyFrame->GetNavState().Get_dBias_Acc());

    cv::Mat Tbc;
    if(mpParams->GetEstimateExtrinsicBetweenCameraAndIMU())     // estimate the Tbc
        mCurrentFrame.UpdatePoseFromNS(mpLocalMapper->GetVINSInitTbc());
    else
        mCurrentFrame.UpdatePoseFromNS(mpParams->GetMatTbc());

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMapWithIMU(bool bMapUpdated)
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // If: Map updated, optimize with last KeyFrame
    if(mpLocalMapper->GetFirstVINSInited() ) //|| bMapUpdated)
    {
        // Get initial pose from Last KeyFrame
        IMUPreintegrator imupreint = GetIMUPreIntSinceLastKF(&mCurrentFrame, mpLastKeyFrame, mvIMUSinceLastKF);

        // Test log
        if(mpLocalMapper->GetFirstVINSInited() && !bMapUpdated) cerr<<"1-FirstVinsInit, but not bMapUpdated. shouldn't"<<endl;
        if(mCurrentFrame.GetNavState().Get_dBias_Acc().norm() > 1e-6) cerr<<"TrackLocalMapWithIMU current Frame dBias acc not zero"<<endl;
        if(mCurrentFrame.GetNavState().Get_dBias_Gyr().norm() > 1e-6) cerr<<"TrackLocalMapWithIMU current Frame dBias gyr not zero"<<endl;

        if(mpParams->GetEstimateExtrinsicBetweenCameraAndIMU())
            Optimizer::PoseOptimization(&mCurrentFrame,mpLastKeyFrame,imupreint,mpLocalMapper->GetGravityVec(), mpLocalMapper->GetVINSInitTbc(), true);
        else
            Optimizer::PoseOptimization(&mCurrentFrame,mpLastKeyFrame,imupreint,mpLocalMapper->GetGravityVec(), mpParams->GetMatTbc(), true);
    }
    else    // else: Map not updated, optimize with last Frame
    {
//        if(mpParams->GetEstimateExtrinsicBetweenCameraAndIMU())
//            Optimizer::PoseOptimization(&mCurrentFrame,&mLastFrame,mIMUPreIntInTrack,mpLocalMapper->GetGravityVec(), mpLocalMapper->GetVINSInitTbc(), true);
//        else
//            Optimizer::PoseOptimization(&mCurrentFrame,&mLastFrame,mIMUPreIntInTrack,mpLocalMapper->GetGravityVec(), mpParams->GetMatTbc(), true);

        Optimizer::PoseOptimization(&mCurrentFrame);
    }

    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

        if(pMP)
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                pMP->IncreaseFound();           // 由于MapPoints可以被当前帧观测到，其被观测统计量加1
                if(!mbOnlyTracking)
                {
                    if(pMP->Observations()>0)   // if 该MapPoint被其它关键帧观测到过
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                pMP = static_cast<MapPoint*>(NULL);
        }

//        if(mCurrentFrame.mvpMapPoints[i])
//        {
//            if(!mCurrentFrame.mvbOutlier[i])
//            {
//                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
//                if(!mbOnlyTracking)
//                {
//                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
//                        mnMatchesInliers++;
//                }
//                else
//                    mnMatchesInliers++;
//            }
//            else if(mSensor==System::STEREO)
//                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

//        }
    }

    // Test log
    {
        if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
            std::cout << "[INFO] TrackLocalMapWithIMU fail due to (mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50), mnMatchesInliers = " << mnMatchesInliers << std::endl;
        if(mnMatchesInliers<30)
            std::cout << "[INFO] TrackLocalMapWithIMU fail due to (mnMatchesInliers<30), mnMatchesInliers = " << mnMatchesInliers << ", bMapUpdated = " << bMapUpdated << ", mpLastKeyFrame->mnFrameId = " << mpLastKeyFrame->mnFrameId << std::endl;
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}

IMUPreintegrator Tracking::GetIMUPreIntSinceLastKF(Frame* pCurF, KeyFrame* pLastKF, const std::vector<IMUData>& vIMUSInceLastKF)
{
    // Reset pre-integrator first
    IMUPreintegrator IMUPreInt;
    IMUPreInt.reset();

    Vector3d bg = pLastKF->GetNavState().Get_BiasGyr();     // get the biasg of Last KeyFrame
    Vector3d ba = pLastKF->GetNavState().Get_BiasAcc();     // get the biasa of Last KeyFrame

    // remember to consider the gap between the last KF and the first IMU
    {
        const IMUData& imu = vIMUSInceLastKF.front();
        double dt = imu._t - pLastKF->mTimeStamp;
        IMUPreInt.update(imu._g - bg, imu._a - ba, dt);

        // Test log
        if(dt < 0)
        {
            cerr<<std::fixed<<std::setprecision(3)<<"dt = "<<dt<<", this KF vs last imu time: "<<pLastKF->mTimeStamp<<" vs "<<imu._t<<endl;
            std::cerr.unsetf ( std::ios::showbase );                // deactivate showbase
        }
    }
    // integrate each imu
    for(size_t i=0; i<vIMUSInceLastKF.size(); i++)
    {
        const IMUData& imu = vIMUSInceLastKF[i];
        double nextt;
        if(i==vIMUSInceLastKF.size()-1)
            nextt = pCurF->mTimeStamp;         // last IMU, next is this KeyFrame
        else
            nextt = vIMUSInceLastKF[i+1]._t;  // regular condition, next is imu data

        // delta time
        double dt = nextt - imu._t;
        // update pre-integrator
        IMUPreInt.update(imu._g - bg, imu._a - ba, dt);

        // Test log
        if(dt <= 0)
        {
            cerr<<std::fixed<<std::setprecision(3)<<"dt = "<<dt<<", this vs next time: "<<imu._t<<" vs "<<nextt<<endl;
            std::cerr.unsetf ( std::ios::showbase );                // deactivate showbase
        }
    }

    return IMUPreInt;
}

IMUPreintegrator Tracking::GetIMUPreIntSinceLastFrame(Frame* pCurF, Frame* pLastF)
{
    // Reset pre-integrator first
    IMUPreintegrator IMUPreInt;
    // IMUPreInt.reset();

    pCurF->ComputeIMUPreIntSinceLastFrame(pLastF,IMUPreInt);

    return IMUPreInt;
}

bool Tracking::GetMonoVIEnable(void)
{
    return mbMonoVIEnable;
}

void Tracking::SetMonoVIEnable(bool flag)
{
    mbMonoVIEnable = flag;
}


/********************************************************************************/
/***************************** for VI-ORB_SLAM2 End *****************************/
/********************************************************************************/


Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    mbMonoVIEnable = false;
    mbCreateNewKFAfterReloc = false;
    mbRelocBiasPrepare = false;

    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

// for Monocular
cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }


    Timer frame_construct_timer;
    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    mTimeOfConstructFrame = frame_construct_timer.runTime_ms();   // record time

    //    cout << "[INFO] Construct Frame time is: " << frame_construct_time.runTime_us()/1000.0 << " ms" << RESET << endl;

//    ///////////////////////////////////////
//    if(!mpLocalMapper->AcceptKeyFrames())
//        std::cout << BLUE"[INFO] 1--mpLocalMapper->AcceptKeyFrames() = false" << ", mCurrentFrame.mnId = " << mCurrentFrame.mnId << RESET << std::endl;
//    else
//        std::cout << GREEN"[INFO] 1--mpLocalMapper->AcceptKeyFrames() = true" << ", mCurrentFrame.mnId = " << mCurrentFrame.mnId << RESET << std::endl;
//    ///////////////////////////////////////


    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);


//    // TODO: for VI-ORB_SLAM: Different operation, according to whether the map is updated
    bool bMapUpdated = false;

    /// Step 1: system initialization
    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
        {
            MonocularInitialization();
            if(mState==OK)
                cout << GREEN"MonocularInitialization success" << RESET << endl;
        }

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }   // End of system initialization
    else    /// Step 2: track new frame
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking) // Default: mbOnlyTracking==false
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)  // if Last frame has been successfully tracked
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                // 检查并更新上一帧被替换的MapPoints
                // 更新Fuse函数和SearchAndFuse函数替换的MapPoints
                CheckReplacedInLastFrame();     // 在形成闭环的时候,会更新KeyFrame与MapPoint之间的关系


                /// Step 2.1: tracking LastFrame or refFrame or relocalization
                /// If the MonoVI mode is enable and VINS has been successfully initialized, then track with IMU
                /// Else track with pure monocular
                if (mbMonoVIEnable && mpLocalMapper->GetVINSInited())     // for VI-ORB_SLAM
                {
                    // Debug log
                    static bool bTmp=true;
                    if(bTmp) {
                        bTmp=false;
                        std::cout << "[INFO] MonoVI mode is enable and VINS has been successfully initialized, track with IMU" << std::endl;
                    }

                    mbTrackWithIMUSuccess = false;
                    mbTrackLocalMapWithIMUSuccess = false;
                    mbTrackReferenceKeyFrameWithIMUSuccess = false;
                    mbTrackWithVisionSuccess = false;
                    mbTrackLocalMapWithVisionSuccess = false;
                    mbTrackReferenceKeyFrameWithVisionSuccess = false;

                    // for VI-ORB_SLAM: Different operation, according to whether the map is updated
                    {
                        if(mpLocalMapper->GetMapUpdateFlagForTracking()){
                            bMapUpdated = true;
                            mpLocalMapper->SetMapUpdateFlagInTracking(false);
                        }
                        if(mpLoopClosing->GetMapUpdateFlagForTracking()){
                            bMapUpdated = true;
                            mpLoopClosing->SetMapUpdateFlagInTracking(false);
                        }
                        if(mCurrentFrame.mnId == mnLastRelocFrameId+20){
                            bMapUpdated = true;
                        }
                    }

                    // if 20 Frames after reloc, track with only vision
                    if(mbRelocBiasPrepare) {
                        bOK = TrackReferenceKeyFrame();
                    }
                    else
                    {
                        {   /// TrackWithIMU
                            Timer time_TrackWithIMU;            /// 6ms ~ 18ms
                            bOK = TrackWithIMU(bMapUpdated);    // 用IMU预计分替换匀速运动模型
                            mbTrackWithIMUSuccess = bOK;
                            // Test log
                            if(!bOK) std::cout << RED"[INFO] TrackWithIMU is fail, mCurrentFrame.mnId = " << mCurrentFrame.mnId
                                               << ", IMU num = " << mCurrentFrame.mvIMUDataSinceLastFrame.size() << RESET << endl;

                            mTimeOfTrackWithIMU = time_TrackWithIMU.runTime_ms();   // record time
                            if(mpParams->GetDispalyTimeStatistic()) std::cout << "[INFO]      TrackWithIMU time: " << mTimeOfTrackWithIMU << " ms" << RESET << std::endl;
                        }

                        {   /// TrackReferenceKeyFrameWithIMU
                            if(!bOK) {
                                bOK = TrackReferenceKeyFrameWithIMU();
                                mbTrackReferenceKeyFrameWithIMUSuccess = bOK;
                            }
                            // Test log
                            if(!bOK) std::cout << RED"[INFO] TrackReferenceKeyFrameWithIMU is fail, mCurrentFrame.mnId = " << mCurrentFrame.mnId << RESET << endl;
                        }

                        /// TODO: Trying to TrackWithMotion() and TrackReferenceKeyFrame() when TrackWithIMU and TrackReferenceKeyFrameWithIMU are both fail
                        if(!mbTrackWithIMUSuccess && !mbTrackReferenceKeyFrameWithIMUSuccess)
                        {
                            bool bOK_Tmp = false;
                            if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                                bOK_Tmp = TrackReferenceKeyFrame();
                            else
                            {
                                bOK_Tmp = TrackWithMotionModel();
                                if(bOK_Tmp) mbTrackWithVisionSuccess = true;
                                if(!bOK_Tmp)
                                {
                                    bOK_Tmp = TrackReferenceKeyFrame();
                                    if(bOK_Tmp) mbTrackReferenceKeyFrameWithVisionSuccess = true;
                                }
                            }

                            // Test log
                            if(bOK_Tmp)
                                std::cout << GREEN"TrackWithIMU and TrackReferenceKeyFrameWithIMU are both fail, but TrackWithMotion() or TrackReferenceKeyFrame() is successful." << RESET  << std::endl;

                            // Use only vision when TrackWithIMU() fail.
                            if(mpParams->GetVisionAidWhenTrackWithIMUFail())
                                bOK = bOK_Tmp;
                        }

                    }

                } // End track with IMU
                else                    // for ORB_SLAM
                {
                    // 运动模型是空的或者刚完成重定位
                    // mCurrentFrame.mnId<mnLastRelocFrameId+2这个判断不应该有
                    // 应该只要mVelocity不为空，就优先选择TrackWithMotionModel
                    // mnLastRelocFrameId上一次重定位的那一帧
                    if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                    {
                        // 将上一帧的位姿作为当前帧的初始位姿
                        // 通过BoW的方式在参考帧中找当前帧特征点的匹配点
                        // 优化每个特征点所对应的3D点的重投影误差即可得到位姿
                        bOK = TrackReferenceKeyFrame();
                        if(!bOK) std::cout << RED"[INFO] mVelocity.empty(), TrackReferenceKeyFrame() is fail" << RESET << endl;
                    }
                    else
                    {

                        // 根据恒速度模型设定当前帧的初始位姿
                        // 通过投影的方式在参考帧中找当前帧特征点的匹配点
                        // 优化每个特征点所对应的3D点的冲投影误差即可得到位姿
                        bOK = TrackWithMotionModel();
                        if(!bOK) std::cout << RED"[INFO] TrackWithMotionModel() is fail, mCurrentFrame.mnId = " << mCurrentFrame.mnId << RESET << endl;

                        if(!bOK)
                            // TrackReferenceKeyFrame是跟踪参考帧，不能根据固定运动速度模型预测当前帧的位姿态，通过bow加速匹配（SearchByBow）
                            // 最后通过优化得到优化后的位姿
                            bOK = TrackReferenceKeyFrame();
                            if(!bOK) std::cout << RED"[INFO] TrackReferenceKeyFrame() is fail, mCurrentFrame.mnId = " << mCurrentFrame.mnId << RESET << endl;
                    }
                }

            }
            else    // mState == LOST: Last Frame is tracking lost
            {

                bOK = Relocalization();
                if (bOK)
                {
                    // Test log
                    cout<<GREEN"Relocalized. id: "<<mCurrentFrame.mnId<<RESET<<endl;
                    if(mbMonoVIEnable && !mpLocalMapper->GetVINSInited()) cerr<<"VINS not inited? why."<<endl;

                    /// for VI-ORB_SLAM, when Relocalization()==true and VINS has been inited, then set mbRelocBiasPrepare=true
                    if(mbMonoVIEnable && mpLocalMapper->GetVINSInited())
                        mbRelocBiasPrepare = true;  // 重定位成功后,将Bias重计算标志置为true
                }
//                else
//                    std::cout << RED"Relocalization() fail." << RESET << std::endl;
            }
        }
        else // Default: not execute
        {
            // Localization Mode: Local Mapping is deactivated
            cout << "[INFO] Warning: Localization Mode: Local Mapping is deactivated." << endl;
            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        ///  Track local map with or without IMU
        if(!mbOnlyTracking)
        {
            if(bOK)
            {
                if(mbMonoVIEnable && !mpParams->GetOnlyTrackLocalMap())  /// for VI-ORB_SLAM
                {
                    if(!mpLocalMapper->GetVINSInited())
                        bOK = TrackLocalMap();          // VINS not inited yet, track with only vision information
                    else
                    {
                        if(mbRelocBiasPrepare)
                            bOK = TrackLocalMap();      // 20 Frames after reloc, track with only vision
                        else{
                            cv::Mat Tcw_rem = mCurrentFrame.mTcw;

                            Timer time_TrackLocalMapWithIMU;          /// 10+ ms ~ 20+ ms
                            bOK = TrackLocalMapWithIMU(bMapUpdated);    // Normal (VI-ORB_SLAM): track with vision and IMU

//                            bOK = TrackLocalMap();

                            mTimeOfTrackLocalMapWithIMU = time_TrackLocalMapWithIMU.runTime_ms();   // record time
                            if(mpParams->GetDispalyTimeStatistic()) std::cout << "[INFO]      TrackLocalMapWithIMU time: " << mTimeOfTrackLocalMapWithIMU << " ms" << RESET << std::endl;

                            // Test log
                            {
                                if(!bOK)
                                {
                                    std::cout << RED"[INFO] TrackLocalMapWithIMU is fail, mnMatchesInliers = " << mnMatchesInliers
                                                   << ", mCurrentFrame.mnId = " << mCurrentFrame.mnId
                                                   << ", mvpLocalKeyFrames.size() = " << mvpLocalKeyFrames.size()
                                                   << ", mvpLocalMapPoints.size() = " << mvpLocalMapPoints.size()
                                                   << ", bMapUpdated = " << bMapUpdated
                                                   << ", IMU num = " << mCurrentFrame.mvIMUDataSinceLastFrame.size() << RESET << std::endl;

                                    cv::Mat Rcw_rem = Tcw_rem.rowRange(0,3).colRange(0,3);
                                    Eigen::Vector3d euler_rem = Converter::toEulerAngles(Rcw_rem);
                                    Eigen::Vector3d Ow_rem = Converter::toVector3d(-Rcw_rem.t()*Tcw_rem.rowRange(0,3).col(3));
                                    std::cout << YELLOW"[INFO] 1-- Before TrackLocalMapWithIMU, current frame euler angle = ["
                                              << euler_rem[0] << ", "
                                              << euler_rem[1] << ", "
                                              << euler_rem[2] << "]"
                                              << ", CameraCenter = ["
                                              << Ow_rem[0] << ", "
                                              << Ow_rem[1] << ", "
                                              << Ow_rem[2] << "]" << RESET << std::endl;

                                    cv::Mat Rcw = mCurrentFrame.GetRotationInverse().t();
                                    Eigen::Vector3d euler = Converter::toEulerAngles(Rcw);
                                    Eigen::Vector3d Ow =  Converter::toVector3d(mCurrentFrame.GetCameraCenter());

                                    std::cout << YELLOW"[INFO] 2-- After TrackLocalMapWithIMU, current frame euler angle = ["
                                              << euler[0] << ", "
                                              << euler[1] << ", "
                                              << euler[2] << "]"
                                              << ", CameraCenter = ["
                                              << Ow[0] << ", "
                                              << Ow[1] << ", "
                                              << Ow[2] << "]" << RESET << std::endl;
                                }

                            }

                            if(!bOK && bMapUpdated)
                            {
                                mCurrentFrame.SetPose(Tcw_rem);
                                bOK = TrackLocalMapWithIMU(false);
                                if(bOK) std::cout << GREEN"[INFO] TrackLocalMapWithIMU(false) is successful, mnMatchesInliers = " << mnMatchesInliers << RESET << std::endl;
                            }

                            // Test log
                            {
                                // If TrackLocalMapWithIMU is fail, then try TrackLocalMap
                                if(!bOK){
                                    mCurrentFrame.SetPose(Tcw_rem);
                                    bool bOK_Tmp = TrackLocalMap();

                                    // Test log
                                    if(bOK_Tmp)
                                    {
                                        std::cout << GREEN"[INFO] TrackLocalMapWithIMU fail but TrackLocalMap is successful. mnMatchesInliers = " << mnMatchesInliers << RESET << std::endl;

                                        cv::Mat Rcw = mCurrentFrame.GetRotationInverse().t();
                                        Eigen::Vector3d euler = Converter::toEulerAngles(Rcw);
                                        Eigen::Vector3d Ow =  Converter::toVector3d(mCurrentFrame.GetCameraCenter());

                                        std::cout << YELLOW"[INFO] 3-- After TrackLocalMap, current frame euler angle = ["
                                                  << euler[0] << ", "
                                                  << euler[1] << ", "
                                                  << euler[2] << "]"
                                                  << ", CameraCenter = ["
                                                  << Ow[0] << ", "
                                                  << Ow[1] << ", "
                                                  << Ow[2] << "]" << RESET << std::endl;

                                    }
                                    else
                                    {
                                        std::cout << RED"[INFO] TrackLocalMapWithIMU fail but TrackLocalMap still fail.  mnMatchesInliers = " << mnMatchesInliers << RESET << std::endl;
                                    }

                                    // Use only vision when TrackLocalMapWithIMU() fail
                                    if(mpParams->GetVisionAidWhenTrackWithIMUFail())
                                        bOK = bOK_Tmp;
                                }
                            }


                        }
                    }
                }
                else    /// for Monocular SLAM
                {
                    bOK = TrackLocalMap();
                }
            }
        }
        else // Default: not execute
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        // If TrackWithIMU()/TrackWithMotionModel()/TrackReferenceKeyFrame()/Relocalization()/TrackLocalMap()/TrackLocalMapWithIMU() is good
        // If tracking is good, check if we need to re-compute the IMU Bias
        if(bOK){
            mState = OK;

            // Add Frames and re-compute IMU bias after reloc

            if(mbMonoVIEnable && mbRelocBiasPrepare){
                mv20FramesReloc.push_back(mCurrentFrame);   // save the Frames after reloc

                // Before creating new keyframe, use 20 consecutive frames to re-compute IMU bias
                // Use 20(mMaxFrames) Frames to re-compute the IMU bias
                /// 存在问题: 若重定位之后的20帧内又tracking lost, mbRelocBiasPrepare不会被重置为false
                /// TODO: clear these buffer when tracking lost
                if(mCurrentFrame.mnId == mnLastRelocFrameId+mMaxFrames-1){
                    NavState nscur;
                    RecomputeIMUBiasAndCurrentNavstate(nscur);  // re-compute Bias and NavState
                    // Update NavState of mCurrentFrame
                    mCurrentFrame.SetNavState(nscur);
                    // Clear flag and Frame buffer
                    mbRelocBiasPrepare = false;
                    mv20FramesReloc.clear();

                    // Release LocalMapping. To ensure to insert new keyframe.
                    mpLocalMapper->Release();
                    // Create new KeyFrame
                    mbCreateNewKFAfterReloc = true;
                    //Debug log
                    cout<<"NavState recomputed."<<endl;
                    cout<<"V:"<<mCurrentFrame.GetNavState().Get_V().transpose()<<endl;
                    cout<<"bg:"<<mCurrentFrame.GetNavState().Get_BiasGyr().transpose()<<endl;
                    cout<<"ba:"<<mCurrentFrame.GetNavState().Get_BiasAcc().transpose()<<endl;
                    cout<<"dbg:"<<mCurrentFrame.GetNavState().Get_dBias_Gyr().transpose()<<endl;
                    cout<<"dba:"<<mCurrentFrame.GetNavState().Get_dBias_Acc().transpose()<<endl;
                }

            }
        }
        else{
            mState=LOST;

            // The tracking maybe LOST within 20 frames after relocalization, so some variables need to be cleared
            if(mbMonoVIEnable){
                if(mv20FramesReloc.size()>0)
                    mv20FramesReloc.clear();        // Clear Frame vectors for reloc bias computation
                if(mbRelocBiasPrepare)
                    mbRelocBiasPrepare = false;     // Disable the mbRelocBiasPrepare flag
            }

        }

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;     // Tcw=Tcl*Tlw ==> Tcl=Tcw*Tlw.inverse(), mVelocity=Tcl
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);


            // Clean VO matches /// for RGB-D and Stereo
            {
                // Step 2.5: clear the mvpMapPoints created by UpdateLastFrame()
                for(int i=0; i<mCurrentFrame.N; i++)
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                    if(pMP)
                        if(pMP->Observations()<1)
                        {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                        }
                }

                // Delete temporal MapPoints
                // 对Monocular或!mbOnlyTracking, mlpTemporalPoints.size() = 0
                // 步骤2.6：清除临时的MapPoints，这些MapPoints在TrackWithMotionModel的UpdateLastFrame函数里生成（仅双目和rgbd）
                // 这里生成的仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中
                // 步骤2.5中只是在当前帧中将这些MapPoints剔除，这里从MapPoints数据库中删除
                for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
                {
                    MapPoint* pMP = *lit;
                    delete pMP;
                }
                // 这里不仅仅是清除mlpTemporalPoints，通过delete pMP还删除了指针指向的MapPoint
                mlpTemporalPoints.clear();
            }

            // Check if we need to insert a new keyframe
            // Force to create a new keyframe if mCurrentFrame is the 20th frame after reloc
            if(NeedNewKeyFrame() || mbCreateNewKFAfterReloc)
            {
                CreateNewKeyFrame();
            }
            // Clear flag
            if(mbCreateNewKFAfterReloc)
                mbCreateNewKFAfterReloc = false;

            // Step 2.8: 删除那些在bundle adjustment中检测为outlier的3D map点
            //           在PoseOptimization时会判断MapPoints是否为outlier
            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.            
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }

            // Clear First-Init flag
            if(mpLocalMapper->GetFirstVINSInited())
            {
                mpLocalMapper->SetFirstVINSInited(false);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }

//            // In MonoVI mode, when tracking lost, reset the system if the VINS hasn't inited.
//            if( mbMonoVIEnable && !mpLocalMapper->GetVINSInited() )
//            {
//                cout << "In MonoVI mode, when tracking lost, reset the system if the VINS hasn't inited." << endl;
//                mpSystem->Reset();
//                return;
//            }

        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }   // End of track new frame

    // Step 3: Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

//    /// Test
//   {
//       Vector3d bg = mCurrentFrame.GetNavState().Get_BiasGyr();
//       Vector3d ba = mCurrentFrame.GetNavState().Get_BiasAcc();

//       Vector3d bg_refK = mCurrentFrame.mpReferenceKF->GetNavState().Get_BiasGyr();
//       Vector3d ba_refK = mCurrentFrame.mpReferenceKF->GetNavState().Get_BiasAcc();
//       cout << "newFrame.id = " << mCurrentFrame.mnId << ", bg.norm() = " << bg.norm() << ", ba.norm() = " << ba.norm()
//           << ", refK.id = " << mCurrentFrame.mpReferenceKF->mnId<<", bg_refK.norm() = " << bg_refK.norm() << ", ba_refK.norm() = " << ba_refK.norm() << endl;
//   }
}


void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

/**
 * @brief 单目的地图初始化
 *
 * 并行地计算基础矩阵和单应性矩阵，选取其中一个模型，恢复出最开始两帧之间的相对姿态以及点云
 * 得到初始两帧的匹配、相对运动、初始MapPoints
 */
void Tracking::MonocularInitialization()
{

    // 如果单目初始器还没有被创建,则创建单目初始器
    if(!mpInitializer)
    {
        // Clear imu data
        if(mbMonoVIEnable){
            if(!mvIMUSinceLastKF.empty())
                mvIMUSinceLastKF.clear();
        }

        // Set Reference Frame
        // 单目初始帧的特征点数必须大于100
        if(mCurrentFrame.mvKeys.size()>100)
        {
            // 步骤1:得到用于初始化的第一帧,初始化需要两帧
            mInitialFrame = Frame(mCurrentFrame);
            // 记录最近的一帧
            mLastFrame = Frame(mCurrentFrame);
            // mvbPrevMatched最大的情况就是所有特征点都被跟踪上
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            // 由当前帧构造初始器 sigma:1.0 iterations:200
            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        // 步骤2:如果当前帧特征点数大于100, 则得到用于初始化的第二帧
        // 如果此时特征点的数量太少,则删除初始化器,等待之后某帧的特征点数量大于100再重新构造初始化器
        // 因此只有连续两帧的特征点个数都大于100时,才能继续进行初始化过程
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        // 步骤3:在mInitialFrame与mCurrentFrame中找匹配的特征点对
        // mvbPrevMatched为前一帧的特征点,存储了mInitialFrame中哪些点将进行接下来的匹配
        // mvIniMatches存储mInitialFrame,mCurrentFrame之间匹配的特征点index( mCurrentFrame特征点的index)
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        // 步骤4：如果初始化的两帧之间的匹配点太少，重新初始化
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated;    // Triangulated Correspondences (mvIniMatches)
                                        // == ture: 匹配点能被三角化(inliers)
                                        // == false: 匹配点不能被三角化(outliers)

        // 步骤5:通过计算H模型或F模型进行单目初始化, 得到两帧间的相对运动与初始MapPoints
        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            // 删除一些无法用于三角化的匹配点
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            // 将初始化的第一帧作为世界坐标系，因此第一帧变换矩阵为单位矩阵
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            // 由Rcw和tcw构造Tcw,并赋值给mTcw，mTcw为世界坐标系到该帧的变换矩阵
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            // 步骤6：将三角化得到的3D点包装成MapPoints
            // Initialize函数会得到mvIniP3D，
            // mvIniP3D是cv::Point3f类型的一个容器，是个存放3D点的临时变量，
            // CreateInitialMapMonocular将3D点包装成MapPoint类型存入KeyFrame和Map中
            CreateInitialMapMonocular();
        }
    }
}

/**
 * @brief CreateInitialMapMonocular
 *
 * 为单目摄像头三角化生成MapPoints
 * mvIniP3D存放匹配点的3D坐标,由Initialize函数计算得到,
 * 本函数将3D点包装成MapPoint类型存入KeyFrame和Map中
 */
void Tracking::CreateInitialMapMonocular()
{
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
    if(mbMonoVIEnable)
    {
        // The first imu package include 2 parts for KF1 and KF2
        vector<IMUData> vimu1, vimu2;
        if(mvIMUSinceLastKF.empty())
        {
            cout << RED"CreateInitialMapMonocular. mvIMUSinceLastKF is empty. This shouldn't happen" << RESET << endl;
            return;
        }

        for(size_t i=0, iend=mvIMUSinceLastKF.size(); i<iend; i++)
        {
            IMUData imu = mvIMUSinceLastKF[i];
            if(imu._t < mInitialFrame.mTimeStamp)
                vimu1.push_back(imu);
            else
                vimu2.push_back(imu);
        }

        pKFini->SetIMUData(vimu1);              // set IMUData for the first KeyFrame
        pKFini->SetPrevAndNextKeyFrame(NULL);   // the PrevKF of the first KeyFrame should be NULL
        pKFini->ComputePreInt();

        pKFcur->SetIMUData(vimu2);              // set IMUData for second KeyFrame
        pKFcur->SetPrevAndNextKeyFrame(pKFini); // the PrevKF of the second KeyFrame is the first KeyFrame
        pKFcur->ComputePreInt();

        mvIMUSinceLastKF.clear();   // remember to clear the IMU buffer
    }

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)  // origin: 100
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        cout << "Finish reseting..." << endl;
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
//    if (!mpLocalMapper->GetVINSInited())
        UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    // Test log
    if(mpLocalMapper->GetVINSInited())
    {
        std::cout <<CYAN "In TrackWithMotionModel(), nmatchesMap = " << nmatchesMap << RESET << std::endl;
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    if(mbMonoVIEnable){
        // Do not insert keyframes if bias is not computed in VINS mode
        if(mbRelocBiasPrepare/* && mpLocalMapper->GetVINSInited()*/)
            return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

//    /// for VI-ORB_SLAM
//    bool cTimeGap = false;
//    if(mbMonoVIEnable){
//        double timegap = 0.1;
//        if(mpLocalMapper->GetVINSInited())
//            timegap = 0.4;      // in VINS mode


////        cTimeGap = (fabs(mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp)>=timegap && mnMatchesInliers>15);
//    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;                           // default; mMaxFrames=fps
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);    // default: mMinFrames=0
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

//    if( ((c1a||c1b||c1c)&&c2) || cTimeGap )
//    if( (c1a||c1b||c1c || cTimeGap)&&c2 )
    if( (c1a||c1b||c1c )&&c2 )
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            std::cout << BLUE"[INFO] mpLocalMapper->InterruptBA();" << RESET << std::endl;
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                // 队列里不能阻塞太多关键帧
                // tracking插入关键帧不是直接插入，而且先插入到mlNewKeyFrames中，
                // 然后localmapper再逐个pop出来插入到mspKeyFrames
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}


void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    // 步骤1: 将当前帧构造成关键帧
    //TODO: is it necessary to clear IMU buffers if this is the first KeyFrame after relocalization (also no prevKF)?
    // Test log
    if (mbCreateNewKFAfterReloc)
    {
        std::cout << RED"mvIMUSinceLastKF.size() = " << mvIMUSinceLastKF.size() << RESET << std::endl;
        std::cout << RED"TODO: Is it necessary to clear IMU buffers if this is the first KeyFrame after relocalization ??" << RESET << std::endl;
    }

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
    if(mbMonoVIEnable){
        pKF->SetIMUData(mvIMUSinceLastKF);              // set IMUData for new KeyFrame
        pKF->SetPrevAndNextKeyFrame(mpLastKeyFrame);    // set PrevKF and NextKF for new KeyFrame

        pKF->SetInitialNavStateAndBias(mCurrentFrame.GetNavState());
        pKF->ComputePreInt();

        mvIMUSinceLastKF.clear();   // remember to clear the IMU buffer
    }

    // 步骤2: 将当前关键帧设置为当前帧的参考关键帧
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    // 这段代码和UpdateLastFrame中的那一部分代码功能相同
    // 步骤3：对于双目或rgbd摄像头，为当前帧生成新的MapPoints
    if(mSensor!=System::MONOCULAR)  // for Stereo & RGB-D
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    // 将pKF插入mlNewKeyFrames, mlNewKeyFrames.empty()==false后,将触发LocalMapping线程主程序
    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;

    mCurrentFrame.SetCurFrameAsKeyframe();
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}

/**
 * @brief 更新局部关键帧，called by UpdateLocalMap()
 *
 * 遍历当前帧的MapPoints，将观测到这些MapPoints的关键帧和相邻的关键帧取出，更新mvpLocalKeyFrames
 */
void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    // 步骤1: 遍历当前帧的MapPoints, 记录所有能观测到当前帧MapPoints的关键帧
    map<KeyFrame*,int> keyframeCounter;     // int 表示KeyFrame观测到当前帧MapPoints的个数
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                // 能观测到pMP的关键帧
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);     // 存储reference keyframe (与当前帧有最多共视点的关键帧)

    // 步骤2: 更新局部关键帧(mvpLocalKeyFrames),添加局部关键帧有三个策略
    // 先清空局部关键帧
    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    // V-D K1: shares the map points with current frame
    // 策略1：能观测到当前帧MapPoints的关键帧作为局部关键帧
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        // mnTrackReferenceForFrame防止重复添加局部关键帧
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    // V-D K2: neighbors to K1 in the covisibility graph
    // 策略2：与策略1得到的局部关键帧共视程度很高的关键帧作为局部关键帧
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        // 策略2.1:最佳共视的10帧
        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                // mnTrackReferenceForFrame防止重复添加局部关键帧
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // 策略2.2:自己的子关键帧
        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // 策略2.3:自己的父关键帧
        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            // mnTrackReferenceForFrame防止重复添加局部关键帧
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;

                if(mbMonoVIEnable)    /// For VI-ORB_SLAM
                {
                    ;
                }
                else
                    break;  /// for Monocular SLAM,
            }
        }

        /// for VI-ORB_SLAM: 将前后关键帧添加进 mvpLocalKeyFrames
        if(mbMonoVIEnable)
        {
            KeyFrame* pPrevKF = pKF->GetPrevKeyFrame();
            if(pPrevKF)
            {
                if(pPrevKF->isBad()) cerr<<"pPrevKF is bad in UpdateLocalKeyFrames()?????"<<endl;
                if(pPrevKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pPrevKF);
                    pPrevKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                }
            }

            KeyFrame* pNextKF = pKF->GetNextKeyFrame();
            if(pNextKF)
            {
                if(pNextKF->isBad()) cerr<<"pNextKF is bad in UpdateLocalKeyFrames()?????"<<endl;
                if(pNextKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNextKF);
                    pNextKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                }
            }
        }

    }

    // V-D Kref： shares the most map points with current frame
    // 步骤3：更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧
    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Step 1: Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    // Step 2: Query KeyFrame Database for keyframe candidates
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper..." << endl;
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

//    if(mbMonoVIEnable)
//    {
//        cout << "Reseting mvIMUSinceLastKF..." << endl;
//        if(!mvIMUSinceLastKF.empty())
//            mvIMUSinceLastKF.clear();
//        cout << " done" << endl;
//    }

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
