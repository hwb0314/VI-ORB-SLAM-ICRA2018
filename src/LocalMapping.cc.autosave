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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Converter.h"
#include<mutex>

namespace ORB_SLAM2
{
/// for VI-ORB_SLAM2
/********************************************************************************/
/**************************** for VI-ORB_SLAM2 Start ****************************/
/********************************************************************************/
bool LocalMapping::SetConfigParam(ConfigParam* pParams)
{
    mpParams = pParams;

    mnLocalWindowSize = mpParams->GetLocalWindowSize();
    std::cout << "mnLocalWindowSize: " << mnLocalWindowSize<<std::endl;

    SetVINSInited(false);
    mbFirstTry = true;
    mbFirstVINSInited = false;

    mbVINSInitRbcConverged = false;
    mbVINSInitPbcConverged = false;

    return true;
}

bool LocalMapping::GetDeactiveLoopCloserInMonoVI(void)
{
    return mbDeactiveLoopCloserInMonoVI;
}

void LocalMapping::SetDeactiveLoopCloserInMonoVI(bool flag)
{
    mbDeactiveLoopCloserInMonoVI = flag;
}

bool LocalMapping::GetMonoVIEnable(void)
{
    return mbMonoVIEnable;
}

void LocalMapping::SetMonoVIEnable(bool flag)
{
    mbMonoVIEnable = flag;
}

void LocalMapping::AddToLocalWindow(KeyFrame *pKF)
{
    mlLocalKeyFrames.push_back(pKF);
    if(mlLocalKeyFrames.size() > mnLocalWindowSize)
        mlLocalKeyFrames.pop_front();   // maintain the size of KF as mnLocalWindowSize
}

void LocalMapping::DeleteBadInLocalWindow(void)
{
    std::list<KeyFrame*>::iterator lit = mlLocalKeyFrames.begin();
    while(lit != mlLocalKeyFrames.end())
    {
        KeyFrame* pKF = *lit;
        //Test log
        if(!pKF) cout<<"pKF null?"<<endl;
        if(pKF->isBad())
        {
            lit = mlLocalKeyFrames.erase(lit);
        }
        else
        {
            lit++;
        }
    }
}

/**
 * @brief LocalMapping::TryInitVIOWithoutPreCalibration
 * @return
 * try to initial the Mono VIO
 */
bool LocalMapping::TryInitVIOWithoutPreCalibration(void)
{
    static double gt_yaw = 0, gt_pitch = 0, gt_roll = 0;
    static double gt_x = 0, gt_y = 0, gt_z = 0;

    /// Step 0. prepare debug .txt
    static bool fopened = false;
    static ofstream fgw, fscale, fbiasa, fbiasg,
            fR_bc_Estimate, fR_bc_gt, fp_bc_Estimate, fp_bc_gt, fp_bc_refined,
            fp_bc_Estimate_without_weight, fR_bc_Estimate_without_weight, fcalibrated_errors_Rbc, fcalibrated_errors_pbc;
    if(!fopened)
    {
        string tmpfilepath = mpParams->getTmpFilePath();
        fgw.open(tmpfilepath+"gw.txt");
        fscale.open(tmpfilepath+"scale.txt");
        fbiasa.open(tmpfilepath+"biasa.txt");;
        fbiasg.open(tmpfilepath+"biasg.txt");
        fR_bc_Estimate.open(tmpfilepath+"R_bc_estimate.txt");
        fR_bc_gt.open(tmpfilepath+"R_bc_groundtruth.txt");
        fp_bc_Estimate.open(tmpfilepath+"p_bc_estimate.txt");
        fp_bc_gt.open(tmpfilepath+"p_bc_groundtruth.txt");
        fp_bc_refined.open(tmpfilepath+"p_bc_refined.txt");

        fp_bc_Estimate_without_weight.open(tmpfilepath+"p_bc_Estimate_without_weight.txt");
        fR_bc_Estimate_without_weight.open(tmpfilepath+"R_bc_Estimate_without_weight.txt");
        fcalibrated_errors_Rbc.open(tmpfilepath+"calibrated_errors_Rbc.txt");
        fcalibrated_errors_pbc.open(tmpfilepath+"calibrated_errors_pbc.txt");


        if(fgw.is_open() && fscale.is_open() && fbiasa.is_open()
                && fbiasg.is_open() && fR_bc_Estimate.is_open()
                && fR_bc_gt.is_open() && fp_bc_Estimate.is_open() && fp_bc_refined.is_open()
                && fp_bc_Estimate_without_weight.is_open() && fR_bc_Estimate_without_weight.is_open()
                && fcalibrated_errors_Rbc.is_open() && fcalibrated_errors_pbc.is_open())
            fopened = true;
        else
        {
            std::cerr << "file open error in TryInitVIOWithoutPreCalibration" << std::endl;
            fopened = false;
        }

        fgw<<std::fixed<<std::setprecision(6);
        fscale<<std::fixed<<std::setprecision(6);
        fbiasa<<std::fixed<<std::setprecision(6);
        fbiasg<<std::fixed<<std::setprecision(6);
        fR_bc_Estimate<<std::fixed<<std::setprecision(6);
        fR_bc_gt<<std::fixed<<std::setprecision(6);
        fp_bc_Estimate<<std::fixed<<std::setprecision(6);
        fp_bc_gt<<std::fixed<<std::setprecision(6);
        fp_bc_refined<<std::fixed<<std::setprecision(6);

        fR_bc_Estimate_without_weight<<std::fixed<<std::setprecision(6);
        fp_bc_Estimate_without_weight<<std::fixed<<std::setprecision(6);

        fcalibrated_errors_Rbc<<std::fixed<<std::setprecision(6);
        fcalibrated_errors_pbc<<std::fixed<<std::setprecision(6);

        fgw << "timestamp gwafter_x gwafter_y gwafter_z gwbefore_x gwbefore_y gwbefore z" << endl;
        fscale << "timestamp s_refined s_star" << endl;

        fbiasa << "timestamp biasa_x biasa_y biasa_z" << endl;
        fbiasg << "timestamp biasg_x biasg_y biasg_z" << endl;

        // Estimation of Rbc: timestamp yaw pitch roll. Units: degree
        fR_bc_Estimate << "timestamp yaw pitch roll" << endl;

        // Estimation of pbc_star (first estimation): timestamp x y zl. Units: meter
        fp_bc_Estimate << "timestamp x y z" << endl;

        // Estimation of pbc_refined (after refinement): timestamp x y zl. Units: meter
        fp_bc_refined << "timestamp x y z" << endl;

        fR_bc_Estimate_without_weight << "timestamp yaw pitch roll" << endl;
        fp_bc_Estimate_without_weight << "timestamp x y z" << endl;

        fcalibrated_errors_Rbc << "timestamp e_yaw e_pitch e_roll" << endl;
        fcalibrated_errors_pbc << "timestamp e_x e_y e_z" << endl;
    }

    // write groundtruth into R_bc_gt.txt and p_bc_gt.txt. only write once
    {
        static bool tt = true;
        if (tt)
        {
            tt = false;

            cv::Mat Tbc = mpParams->GetMatTbc();
            cv::Mat Rbc = Tbc.rowRange(0,3).colRange(0,3);
            cv::Mat pbc = Tbc.rowRange(0,3).col(3);

            Eigen::Matrix3d R_bc_gt = Converter::toMatrix3d(Rbc);
            Eigen::Vector3d euler_bc_gt = R_bc_gt.eulerAngles(2, 1, 0);   // yaw, pitch, roll

            gt_yaw = euler_bc_gt(0)* 180 / M_PI;
            gt_pitch = euler_bc_gt(1)* 180 / M_PI;
            gt_roll = euler_bc_gt(2)* 180 / M_PI;

            // GroundTruth of R_bc: yaw pitch roll. Units: degree" << endl
            fR_bc_gt << "yaw_gt pitch_gt roll_gt" << endl;
            fR_bc_gt << gt_yaw << " " << gt_pitch << " " << gt_roll << endl;
            fR_bc_gt.close();

            // GroundTruth of p_bc: x y z. Units: meter
            gt_x = pbc.at<float>(0,0);
            gt_y = pbc.at<float>(1,0);
            gt_z = pbc.at<float>(2,0);

            fp_bc_gt << "x_gt y_gt z_gt" << endl;
            fp_bc_gt << pbc.at<float>(0,0) << " " << pbc.at<float>(1,0) << " " << pbc.at<float>(2,0) << endl;
            fp_bc_gt.close();
        }
    }

    // estimated Extrinsics between IMU and camera
    Vector3d bgest(0,0,0);
    cv::Mat Rcbstar = cv::Mat::eye(3,3,CV_32F);
    double sstar = 0.0;
    cv::Mat gwstar = cv::Mat::zeros(3,1,CV_32F);
    cv::Mat pcbstar = cv::Mat::zeros(3,1,CV_32F);

    // refined results
    double s_refined = 0.0;
    cv::Mat dthetaxy = cv::Mat::zeros(2,1,CV_32F);
    cv::Mat dtheta = cv::Mat::zeros(3,1,CV_32F);
    cv::Mat biasa_ = cv::Mat::zeros(3,1,CV_32F);
    cv::Mat gw_refined = cv::Mat::zeros(3,1,CV_32F);
    cv::Mat pcb_refined = cv::Mat::zeros(3,1,CV_32F);

    // last results
    // create random Rbc
    static bool createRandomRbc = true;
    Eigen::Matrix3d rotationMatrixRand = Eigen::Matrix3d::Identity();

    if(createRandomRbc && mpParams->GetCreateRandomMatrixLastForFirstVIOInit())
    {
        createRandomRbc = false;

        std::srand(time(NULL));
        double yaw_rng      = (std::rand()%1000/1000.0 - 0.5) * 2;  // (-1, 1)
        double pitch_rng    = (std::rand()%1000/1000.0 - 0.5) * 2;  // (-1, 1)
        double roll_rng     = (std::rand()%1000/1000.0 - 0.5) * 2;  // (-1, 1)

        Eigen::AngleAxisd yawAngle(yaw_rng*M_PI, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd pitchAngle(pitch_rng*M_PI, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rollAngle(roll_rng*M_PI, Eigen::Vector3d::UnitX());
        Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
        rotationMatrixRand = q.matrix();

        Eigen::Vector3d euler_angles = rotationMatrixRand.eulerAngles(2, 1, 0); // ZYX顺序,即yaw, pitch, roll顺序
        cout << "\n[INFO] create random Rbc: yaw pitch roll = " << euler_angles.transpose()  << endl;
    }

    static cv::Mat Rcbstar_last = Converter::toCvMat(rotationMatrixRand);   // random
//    static cv::Mat Rcbstar_last = cv::Mat::eye(3,3,CV_32F);           // Identity

    static double sstar_last = -1;
    static cv::Mat gwstar_last = cv::Mat::zeros(3,1,CV_32F);
    static cv::Mat pcbstar_last = cv::Mat::zeros(3,1,CV_32F);

    static double s_refined_last = -1;
    static cv::Mat dtheta_last = cv::Mat::zeros(3,1,CV_32F);;
    static cv::Mat biasa_last = cv::Mat::zeros(3,1,CV_32F);;
    static cv::Mat pcb_refined_last = cv::Mat::zeros(3,1,CV_32F);


    // use all KeyFrames in map to compute
    vector<KeyFrame*> vScaleGravityKF = mpMap->GetAllKeyFrames();
    int N = vScaleGravityKF.size();

    // only when the map has enough keyframes, can the system try to initialize
    if(mpMap->KeyFramesInMap()<=mnLocalWindowSize)  // Default: mnLocalWindowSize = 10
        return false;

    // record start time
    if(mbFirstTry)
    {
       mbFirstTry = false;
       mnStartTime = mpCurrentKeyFrame->mTimeStamp;
    }

    /// Step 1. IV-A Gyroscope Bias and Rbc Estimation
    /// Iterative estimate gyro biasg and Rbc until the Rbc is convergent.
    /// Note: In practice we estimate the Rcb and pcb, so Rbc = Rcb.t(), pbc = -Rcb.t()*pcb

    if(sstar_last==-1)  // performa once, assume biasg=0, and estimate Rcbstar_last
    {
        cout << "// performa once, assume biasg=0, and estimate Rcbstar_last" << endl;
        sstar_last = -2;
        /// My innovation 1
        /// Step 1.2: Estimate Rcb (the rotation from body to camera)
        {
            Eigen::Quaterniond q_cb_est;
            static Eigen::Quaterniond q_cb_est_last = Eigen::Quaterniond(1,0,0,0);  // init as a unit quaternion
            static bool bFirstEstimateRcb = true;

            // Solve A*q=0 for q=[w,x,y,z]^T
            // We can get N-1 equations
            cv::Mat A = cv::Mat::zeros(4*(N-1),4,CV_32F);
            cv::Mat I3 = cv::Mat::eye(3,3,CV_32F);
    //        cv::Mat A_without_weight = cv::Mat::zeros(4*(N-1),4,CV_32F);    // just for Debug

            /// Step 1.2.1: construct A
            for(int i=0; i<N-1; i++)
            {
                KeyFrame* pKF1 = vScaleGravityKF[i];
                KeyFrame* pKF2 = vScaleGravityKF[i+1];
                // Step 1.2.1-a: Compute the delta_R_B and corresponding quaternion q_B (normalized)
                Eigen::Matrix3d delta_R_B = pKF2->GetIMUPreInt().getDeltaR();
                Eigen::Quaterniond q_B = Eigen::Quaterniond(delta_R_B); q_B.norm();    // Note: the order is (x,y,z,w)
                // Step 1.2.1-b: Compute the delta_R_C and corresponding quaternion q_C (normalized)
                Eigen::Matrix3d delta_R_C = Converter::toMatrix3d(pKF1->GetRotation() * pKF2->GetRotation().t());
                Eigen::Quaterniond q_C = Eigen::Quaterniond(delta_R_C); q_C.norm();    // Note: the order is (x,y,z,w)

                // Step 1.2.1-c: Compute the matrix Q
                Eigen::Vector3d q_B_v(q_B.x(), q_B.y(), q_B.z());
                double q_B_w = q_B.w();
                Eigen::Vector3d q_C_v(q_C.x(), q_C.y(), q_C.z());
                double q_C_w = q_C.w();

                cv::Mat Q = cv::Mat::zeros(4, 4, CV_32F);
                Q.at<float>(0,0) = q_B_w - q_C_w;
                Eigen::Vector3d tmp_v(q_B_v-q_C_v);
                cv::Mat Q_10_30 = Converter::toCvMat( Eigen::Matrix<double,3,1>(tmp_v(0), tmp_v(1), tmp_v(2)));
                Q_10_30.copyTo(Q.rowRange(1,4).col(0));
                cv::Mat Q_01_03= -Q_10_30.t();
                Q_01_03.copyTo(Q.row(0).colRange(1,4));
                cv::Mat Q_11_33 = (q_B_w - q_C_w)*I3-Converter::toSkew(q_B_v)-Converter::toSkew(q_C_v);
                Q_11_33.copyTo(Q.rowRange(1,4).colRange(1,4));

    //            // Debug Test: without weight
    //            Q.copyTo(A_without_weight.rowRange(4*i+0, 4*i+4).colRange(0,4));

                // Step 1.2.1-d: Compute the wight wi
                if(!bFirstEstimateRcb){
                    // error
                    cv::Mat e = Q * ( cv::Mat_<float>(4,1) << q_cb_est_last.w(), q_cb_est_last.x(), q_cb_est_last.y(), q_cb_est_last.z() );
                    // exponential weight function
                    double wi = std::exp(- cv::norm(e) * 200.0);
                    if (cv::norm(e) > 0.05)   // can be deleted
                        wi = 0.0;

                    Q = wi * Q;
                }

                // Step 1.2.1-e: Stack to matrix A
                Q.copyTo(A.rowRange(4*i+0, 4*i+4).colRange(0,4));
            }

            /*{   // Debug Test: without weight
                cv::Mat q_cb_solved_ww;
                cv::SVD::solveZ(A_without_weight,q_cb_solved_ww);
                if(q_cb_solved_ww.at<float>(0,0)< 1.0e-10) q_cb_solved_ww = -q_cb_solved_ww;
                Eigen::Quaterniond q_cb_est_ww = Eigen::Quaterniond(q_cb_solved_ww.at<float>(0,0), q_cb_solved_ww.at<float>(1,0), q_cb_solved_ww.at<float>(2,0), q_cb_solved_ww.at<float>(3,0));
                q_cb_est_ww.norm();;

                Eigen::Matrix3d Rcb_est_matrix_ww = q_cb_est_ww.toRotationMatrix();

                Eigen::Matrix3d Rbc_est_matrix_ww = Rcb_est_matrix_ww.inverse();
                Eigen::Vector3d euler_bc_est_ww = Rbc_est_matrix_ww.eulerAngles(2, 1, 0);   // yaw, pitch, roll
                fR_bc_Estimate_without_weight << mpCurrentKeyFrame->mTimeStamp<<" "
                               << euler_bc_est_ww(0)* 180 / M_PI << " " << euler_bc_est_ww(1) * 180 / M_PI << " " << euler_bc_est_ww(2) * 180 / M_PI << endl;
            }*/

            /// Step 1.2.2: Solve A*q_cb=0  ==> q_cb = argmin|A*q_cb|, with constrain: |q_cb|=1
            cv::Mat q_cb_solved;   // the order is [w,x,y,z]
            cv::SVD::solveZ(A,q_cb_solved);

            // Debug log
            if(q_cb_solved.empty()){
                std::cerr << "[Warning] cv::SVD::solveZ() result is empty" << std::endl;
                return false;
            }
            if(q_cb_solved.at<float>(0,0)< 1.0e-10) q_cb_solved = -q_cb_solved;

            // ref in Eigen/Quaternion.h
            // inline Quaternion(const Scalar& w, const Scalar& x, const Scalar& y, const Scalar& z) : m_coeffs(x, y, z, w){}
            q_cb_est = Eigen::Quaterniond(q_cb_solved.at<float>(0,0), q_cb_solved.at<float>(1,0), q_cb_solved.at<float>(2,0), q_cb_solved.at<float>(3,0));
            q_cb_est.norm();

            q_cb_est_last = q_cb_est;

            /// Step 1.2.3: get Rcbstar, and check convergence
            /// If the rotate euler angles of three axises (yaw, pitch, roll) are both convergent,
            /// we consider the Rcbstar is convergent
            Eigen::Matrix3d Rcb_est_matrix = q_cb_est.toRotationMatrix();
            Rcbstar = Converter::toCvMat(Rcb_est_matrix);

            Rcbstar_last = Rcbstar;

            // write R_bc_estimate.txt and check convergence
            {
                Eigen::Matrix3d Rbc_est_matrix = Rcb_est_matrix.inverse();
                Eigen::Vector3d euler_bc_est = Rbc_est_matrix.eulerAngles(2, 1, 0);   // yaw, pitch, roll
                fR_bc_Estimate << mpCurrentKeyFrame->mTimeStamp<<" "
                               << euler_bc_est(0)* 180 / M_PI << " " << euler_bc_est(1) * 180 / M_PI << " " << euler_bc_est(2) * 180 / M_PI << endl;

                fcalibrated_errors_Rbc << mpCurrentKeyFrame->mTimeStamp<<" "
                                       << gt_yaw-euler_bc_est(0)* 180 / M_PI << " " << gt_pitch-euler_bc_est(1) * 180 / M_PI << " " << gt_roll-euler_bc_est(2) * 180 / M_PI << endl;

                // check whether the estimated Rbc has converged
                {
                    // record estimated results and timestamps
                    mvRecordEstimatedRbc.push_back(euler_bc_est);
                    mvRecordEstimatedRbcTimeStamps.push_back(mpCurrentKeyFrame->mTimeStamp);

                    mbVINSInitRbcConverged = CheckRbcEstimationConverge();
    //                // If the Rbc do not converge, then reset Biasg to zero and re-compute pre-integration
    //                if(!mbVINSInitRbcConverged){
    //                    ResetBiasgToZero(vScaleGravityKF);
    //                    return false;
    //                }

                    // Debug log
                    // std::cout << GREEN"Estimated Rbc has converged." << RESET << std::endl;

                }
            }
        }   // End of Step 1.2: Estimate Rcb (the rotation from body to camera)

    }


    /// Step 1.1. Try to compute initial gyro bias, using optimization with Levenberg.
    bgest = Optimizer::OptimizeInitialGyroBias(vScaleGravityKF, Rcbstar_last);     // use Rcbstar_last as input
    // bgest = Optimizer::OptimizeInitialGyroBias(vScaleGravityKF, mpParams->GetMatTbc().rowRange(0,3).colRange(0,3).t()); // use pre-calib Rcb (=Rbc.t()) as input

    // Update biasg and pre-integration in LocalWindow. Remember to reset back to zero if initialization fail
    for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        pKF->SetNavStateBiasGyr(bgest);
    }
    for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        pKF->ComputePreInt();   // compute the pre-integration, using the estimated biasg_est
    }

    /// My innovation 1
    /// Step 1.2: Estimate Rcb (the rotation from body to camera)
    {
        Eigen::Quaterniond q_cb_est;
        static Eigen::Quaterniond q_cb_est_last = Eigen::Quaterniond(1,0,0,0);  // init as a unit quaternion
        static bool bFirstEstimateRcb = true;

        // Solve A*q=0 for q=[w,x,y,z]^T
        // We can get N-1 equations
        cv::Mat A = cv::Mat::zeros(4*(N-1),4,CV_32F);
        cv::Mat I3 = cv::Mat::eye(3,3,CV_32F);
//        cv::Mat A_without_weight = cv::Mat::zeros(4*(N-1),4,CV_32F);    // just for Debug

        /// Step 1.2.1: construct A
        for(int i=0; i<N-1; i++)
        {
            KeyFrame* pKF1 = vScaleGravityKF[i];
            KeyFrame* pKF2 = vScaleGravityKF[i+1];
            // Step 1.2.1-a: Compute the delta_R_B and corresponding quaternion q_B (normalized)
            Eigen::Matrix3d delta_R_B = pKF2->GetIMUPreInt().getDeltaR();
            Eigen::Quaterniond q_B = Eigen::Quaterniond(delta_R_B); q_B.norm();    // Note: the order is (x,y,z,w)
            // Step 1.2.1-b: Compute the delta_R_C and corresponding quaternion q_C (normalized)
            Eigen::Matrix3d delta_R_C = Converter::toMatrix3d(pKF1->GetRotation() * pKF2->GetRotation().t());
            Eigen::Quaterniond q_C = Eigen::Quaterniond(delta_R_C); q_C.norm();    // Note: the order is (x,y,z,w)

            // Step 1.2.1-c: Compute the matrix Q
            Eigen::Vector3d q_B_v(q_B.x(), q_B.y(), q_B.z());
            double q_B_w = q_B.w();
            Eigen::Vector3d q_C_v(q_C.x(), q_C.y(), q_C.z());
            double q_C_w = q_C.w();

            cv::Mat Q = cv::Mat::zeros(4, 4, CV_32F);
            Q.at<float>(0,0) = q_B_w - q_C_w;
            Eigen::Vector3d tmp_v(q_B_v-q_C_v);
            cv::Mat Q_10_30 = Converter::toCvMat( Eigen::Matrix<double,3,1>(tmp_v(0), tmp_v(1), tmp_v(2)));
            Q_10_30.copyTo(Q.rowRange(1,4).col(0));
            cv::Mat Q_01_03= -Q_10_30.t();
            Q_01_03.copyTo(Q.row(0).colRange(1,4));
            cv::Mat Q_11_33 = (q_B_w - q_C_w)*I3-Converter::toSkew(q_B_v)-Converter::toSkew(q_C_v);
            Q_11_33.copyTo(Q.rowRange(1,4).colRange(1,4));

//            // Debug Test: without weight
//            Q.copyTo(A_without_weight.rowRange(4*i+0, 4*i+4).colRange(0,4));

            // Step 1.2.1-d: Compute the wight wi
            if(!bFirstEstimateRcb){
                // error
                cv::Mat e = Q * ( cv::Mat_<float>(4,1) << q_cb_est_last.w(), q_cb_est_last.x(), q_cb_est_last.y(), q_cb_est_last.z() );
                // exponential weight function
                double wi = std::exp(- cv::norm(e) * 200.0);
                if (cv::norm(e) > 0.05)   // can be deleted
                    wi = 0.0;

                Q = wi * Q;
            }

            // Step 1.2.1-e: Stack to matrix A
            Q.copyTo(A.rowRange(4*i+0, 4*i+4).colRange(0,4));
        }

        /*{   // Debug Test: without weight
            cv::Mat q_cb_solved_ww;
            cv::SVD::solveZ(A_without_weight,q_cb_solved_ww);
            if(q_cb_solved_ww.at<float>(0,0)< 1.0e-10) q_cb_solved_ww = -q_cb_solved_ww;
            Eigen::Quaterniond q_cb_est_ww = Eigen::Quaterniond(q_cb_solved_ww.at<float>(0,0), q_cb_solved_ww.at<float>(1,0), q_cb_solved_ww.at<float>(2,0), q_cb_solved_ww.at<float>(3,0));
            q_cb_est_ww.norm();;

            Eigen::Matrix3d Rcb_est_matrix_ww = q_cb_est_ww.toRotationMatrix();

            Eigen::Matrix3d Rbc_est_matrix_ww = Rcb_est_matrix_ww.inverse();
            Eigen::Vector3d euler_bc_est_ww = Rbc_est_matrix_ww.eulerAngles(2, 1, 0);   // yaw, pitch, roll
            fR_bc_Estimate_without_weight << mpCurrentKeyFrame->mTimeStamp<<" "
                           << euler_bc_est_ww(0)* 180 / M_PI << " " << euler_bc_est_ww(1) * 180 / M_PI << " " << euler_bc_est_ww(2) * 180 / M_PI << endl;
        }*/

        /// Step 1.2.2: Solve A*q_cb=0  ==> q_cb = argmin|A*q_cb|, with constrain: |q_cb|=1
        cv::Mat q_cb_solved;   // the order is [w,x,y,z]
        cv::SVD::solveZ(A,q_cb_solved);

        // Debug log
        if(q_cb_solved.empty()){
            std::cerr << "[Warning] cv::SVD::solveZ() result is empty" << std::endl;
            return false;
        }
        if(q_cb_solved.at<float>(0,0)< 1.0e-10) q_cb_solved = -q_cb_solved;

        // ref in Eigen/Quaternion.h
        // inline Quaternion(const Scalar& w, const Scalar& x, const Scalar& y, const Scalar& z) : m_coeffs(x, y, z, w){}
        q_cb_est = Eigen::Quaterniond(q_cb_solved.at<float>(0,0), q_cb_solved.at<float>(1,0), q_cb_solved.at<float>(2,0), q_cb_solved.at<float>(3,0));
        q_cb_est.norm();

        q_cb_est_last = q_cb_est;

        /// Step 1.2.3: get Rcbstar, and check convergence
        /// If the rotate euler angles of three axises (yaw, pitch, roll) are both convergent,
        /// we consider the Rcbstar is convergent
        Eigen::Matrix3d Rcb_est_matrix = q_cb_est.toRotationMatrix();
        Rcbstar = Converter::toCvMat(Rcb_est_matrix);

        Rcbstar_last = Rcbstar;

        // write R_bc_estimate.txt and check convergence
        {
            Eigen::Matrix3d Rbc_est_matrix = Rcb_est_matrix.inverse();
            Eigen::Vector3d euler_bc_est = Rbc_est_matrix.eulerAngles(2, 1, 0);   // yaw, pitch, roll
            fR_bc_Estimate << mpCurrentKeyFrame->mTimeStamp<<" "
                           << euler_bc_est(0)* 180 / M_PI << " " << euler_bc_est(1) * 180 / M_PI << " " << euler_bc_est(2) * 180 / M_PI << endl;

            fcalibrated_errors_Rbc << mpCurrentKeyFrame->mTimeStamp<<" "
                                   << gt_yaw-euler_bc_est(0)* 180 / M_PI << " " << gt_pitch-euler_bc_est(1) * 180 / M_PI << " " << gt_roll-euler_bc_est(2) * 180 / M_PI << endl;

            // check whether the estimated Rbc has converged
            {
                // record estimated results and timestamps
                mvRecordEstimatedRbc.push_back(euler_bc_est);
                mvRecordEstimatedRbcTimeStamps.push_back(mpCurrentKeyFrame->mTimeStamp);

                mbVINSInitRbcConverged = CheckRbcEstimationConverge();
//                // If the Rbc do not converge, then reset Biasg to zero and re-compute pre-integration
//                if(!mbVINSInitRbcConverged){
//                    ResetBiasgToZero(vScaleGravityKF);
//                    return false;
//                }

                // Debug log
                // std::cout << GREEN"Estimated Rbc has converged." << RESET << std::endl;

            }
        }
    }   // End of Step 1.2: Estimate Rcb (the rotation from body to camera)

    //
    // Only when the Rcb converge, can the following code be executed.
    //

    /// My innovation 2
    /// Step 1.3: Scale, Gravity and pcb (no accelerometer bias) Estimation
    /// Approx Scale and Gravity vector in 'world' frame (first KF's camera frame)
    {
        // Solve A*x=B for x=[s,gw, pcb] 7x1 vector
        cv::Mat A = cv::Mat::zeros(3*(N-2),7,CV_32F);
        cv::Mat B = cv::Mat::zeros(3*(N-2),1,CV_32F);
        cv::Mat I3 = cv::Mat::eye(3,3,CV_32F);

        cv::Mat A_without_weight = cv::Mat::zeros(3*(N-2),7,CV_32F);    // Just for Debug
        cv::Mat B_without_weight = cv::Mat::zeros(3*(N-2),1,CV_32F);    // Just for Debug

        /// Step 1.3.1: Construct A
        for(int i=0; i<N-2; i++)
        {
            KeyFrame* pKF1 = vScaleGravityKF[i];
            KeyFrame* pKF2 = vScaleGravityKF[i+1];
            KeyFrame* pKF3 = vScaleGravityKF[i+2];
            // Delta time between frames
            double dt12 = pKF2->GetIMUPreInt().getDeltaTime();
            double dt23 = pKF3->GetIMUPreInt().getDeltaTime();
            // Pre-integrated measurements
            cv::Mat dp12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaP());
            cv::Mat dv12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaV());
            cv::Mat dp23 = Converter::toCvMat(pKF3->GetIMUPreInt().getDeltaP());
            // Test log
            if(dt12!=pKF2->mTimeStamp-pKF1->mTimeStamp) cerr<<"dt12 != pKF2->mTimeStamp-pKF1->mTimeStamp"<<endl;
            if(dt23!=pKF3->mTimeStamp-pKF2->mTimeStamp) cerr<<"dt23 != pKF3->mTimeStamp-pKF2->mTimeStamp"<<endl;

            // Pose of camera in world frame
            cv::Mat Twc1 = pKF1->GetPoseInverse();
            cv::Mat Twc2 = pKF2->GetPoseInverse();
            cv::Mat Twc3 = pKF3->GetPoseInverse();
            // Position of camera center
            cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
            cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
            cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
            // Rotation of camera, Rwc
            cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
            cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
            cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);
//            // Rotation of IMU, Rwb
//            cv::Mat Rwb1 = Converter::toCvMat( pKF1->GetNavState().Get_RotMatrix() );
//            cv::Mat Rwb2 = Converter::toCvMat( pKF2->GetNavState().Get_RotMatrix() );

            // Stack to A/B matrix
            // lambda*s + beta*g + phi*pcb = gamma
            cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
            cv::Mat beta = 0.5*I3*(dt12*dt12*dt23 + dt12*dt23*dt23);
            cv::Mat phi = (Rc2-Rc3)*dt12 - (Rc1-Rc2)*dt23;
            cv::Mat gamma = Rc1*Rcbstar*dp12*dt23 - Rc2*Rcbstar*dp23*dt12 - Rc1*Rcbstar*dv12*dt12*dt23; // use the estimated Rcbstar
//            cv::Mat gamma = Rwb1*dp12*dt23 - Rwb2*dp23*dt12 - Rwb1*dv12*dt12*dt23; // use the NavState. <-- The result is not good due to the biasa is not considered.

//            {   // Debug Test: without weight
//                lambda.copyTo(A_without_weight.rowRange(3*i+0,3*i+3).col(0));
//                beta.copyTo(A_without_weight.rowRange(3*i+0,3*i+3).colRange(1,4));
//                phi.copyTo(A_without_weight.rowRange(3*i+0,3*i+3).colRange(4,7));
//                gamma.copyTo(B_without_weight.rowRange(3*i+0,3*i+3));
//            }


            // compute the weight wi
            if(sstar_last>0){
                // predict gamma
                cv::Mat gamma_pred = lambda * sstar_last + beta * gwstar_last + phi * pcbstar_last;
                // error
                cv::Mat e = gamma_pred - gamma;
                // exponential weight function
                double  wi = std::exp(- cv::norm(e) * 100.0);
                if (cv::norm(e) > 0.05)   // can be deleted
                    wi = 0.0;

                lambda *= wi;
                beta *= wi;
                phi *= wi;
                gamma *= wi;
            }

            // Stack to matrix A
            lambda.copyTo(A.rowRange(3*i+0,3*i+3).col(0));
            beta.copyTo(A.rowRange(3*i+0,3*i+3).colRange(1,4));
            phi.copyTo(A.rowRange(3*i+0,3*i+3).colRange(4,7));
            gamma.copyTo(B.rowRange(3*i+0,3*i+3));

        }

//        {   // Debug Test: without weight
//            cv::Mat x_ww;
//            cv::solve(A_without_weight,B_without_weight,x_ww,cv::DECOMP_LU|cv::DECOMP_NORMAL);
//            double s_ww = x_ww.at<float>(0);
//            cv::Mat gw_ww = x_ww.rowRange(1,4);   // gravity should be about ~9.8
//            cv::Mat pcb_ww = x_ww.rowRange(4,7);
//            cv::Mat pbc_ww = -Rcbstar.t()*pcb_ww;
//            fp_bc_Estimate_without_weight << mpCurrentKeyFrame->mTimeStamp<<" "
//                           << pbc_ww.at<float>(0) << " " << pbc_ww.at<float>(1) << " " << pbc_ww.at<float>(2) << endl;
//        }


        // Step 1.3.2: Solve Ax=B ==> x* = argmin |A*x-B|
        cv::Mat x;
//        cv::solve(A,B,x,cv::DECOMP_SVD);
        cv::solve(A,B,x,cv::DECOMP_LU|cv::DECOMP_NORMAL);

        // Debuug log
        if(x.empty()){
            std::cerr << "[Warning] cv::solve() result is empty" << std::endl;
            return false;
        }

        sstar = x.at<float>(0);     // scale should be positive
        // Debug log
        if(sstar <= 1.0e-10)    {std::cerr << "[Warning] Scale should be positive, but sstar < 0, why?" << std::endl;}
        gwstar = x.rowRange(1,4);   // |gwstar| should be about ~9.8
        pcbstar = x.rowRange(4,7);

        // write p_bc_estimate.txt
        {
            cv::Mat pbcstar = -Rcbstar.t()*pcbstar;   // use estimated Rcb_est
            fp_bc_Estimate << mpCurrentKeyFrame->mTimeStamp<<" "
                           << pbcstar.at<float>(0) << " " << pbcstar.at<float>(1) << " " << pbcstar.at<float>(2) << endl;
        }

        // Update last record
        sstar_last = sstar;
        gwstar_last = gwstar;
        pcbstar_last = pcbstar;

    }   // End of Step 1.3: Scale, Gravity and pcb (no accelerometer bias) Estimation

//std::vector<cv::Mat> vErrors;
//std::vector<double> vNorms;
//std::vector<double> vTimestamps;
//std::vector<double> vWi;

    /// My innovation 3:
    /// Step 1.4: Accelerometer Bias Estimation, and Scale, Gravity Direction and Pcb Refinement
    {
        // Use gravity magnitude 9.8 as constraint
        cv::Mat GI = cv::Mat::zeros(3,1,CV_32F);
        GI.at<float>(2) = -1 * ConfigParam::GetG();//9.810;
        cv::Mat GIxgwstar = GI.cross(gwstar);
        // vhat = (GI x gwstat) / |GI x gwstar|
        cv::Mat vhat = GIxgwstar / cv::norm(GIxgwstar);
        // theta = atan2(|GI x gwstar|, GI dot gwstar)
        double theta = std::atan2(cv::norm(GIxgwstar), GI.dot(gwstar));

        double theta2 = std::acos(GI.dot(gwstar));
        cout<<"2: vhat: "<<vhat.t()<<", theta: "<<theta*180.0/M_PI<<", theta2: "<<theta2*180.0/M_PI<<endl;


        Eigen::Vector3d vhateig = Converter::toVector3d(vhat);
        Eigen::Matrix3d RWIeig = Sophus::SO3::exp(vhateig*theta).matrix();
        cv::Mat Rwi = Converter::toCvMat(RWIeig);

        // Solve C*x=D for x=[s,dthetaxy,ba] (1+2+3)x1 vector
        cv::Mat C = cv::Mat::zeros(3*(N-2),9,CV_32F);
        cv::Mat D = cv::Mat::zeros(3*(N-2),1,CV_32F);

        for(int i=0; i<N-2; i++)
        {
            KeyFrame* pKF1 = vScaleGravityKF[i];
            KeyFrame* pKF2 = vScaleGravityKF[i+1];
            KeyFrame* pKF3 = vScaleGravityKF[i+2];
            // Delta time between frames
            double dt12 = pKF2->GetIMUPreInt().getDeltaTime();
            double dt23 = pKF3->GetIMUPreInt().getDeltaTime();
            // Pre-integrated measurements
            cv::Mat dp12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaP());
            cv::Mat dv12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaV());
            cv::Mat dp23 = Converter::toCvMat(pKF3->GetIMUPreInt().getDeltaP());
            cv::Mat Jpba12 = Converter::toCvMat(pKF2->GetIMUPreInt().getJPBiasa());
            cv::Mat Jvba12 = Converter::toCvMat(pKF2->GetIMUPreInt().getJVBiasa()); // ??
            cv::Mat Jpba23 = Converter::toCvMat(pKF3->GetIMUPreInt().getJPBiasa());
            // Pose of camera in world frame
            cv::Mat Twc1 = pKF1->GetPoseInverse();
            cv::Mat Twc2 = pKF2->GetPoseInverse();
            cv::Mat Twc3 = pKF3->GetPoseInverse();
            // Position of camera center
            cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
            cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
            cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
            // Rotation of camera, Rwc
            cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
            cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
            cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);
            // Stack to C/D matrix
            // lambda*s + phi*dthetaxy + zeta*ba = psi
            cv::Mat lambda = (pc2-pc1)*dt23 - (pc3-pc2)*dt12;
            cv::Mat phi = - 0.5*(dt12*dt12*dt23 + dt12*dt23*dt23)*Rwi*Converter::toSkew(GI);
            cv::Mat zeta = Rc2*Rcbstar*Jpba23*dt12 - Rc1*Rcbstar*Jpba12*dt23 + Rc1*Rcbstar*Jvba12*dt12*dt23;
            cv::Mat ksi = (Rc2-Rc3)*dt12 - (Rc1-Rc2)*dt23;
            cv::Mat psi =  Rc1*Rcbstar*dp12*dt23 - Rc2*Rcbstar*dp23*dt12 - Rc1*Rcbstar*dv12*dt23*dt12 - 0.5*Rwi*GI*(dt12*dt12*dt23 + dt12*dt23*dt23); // note:  - paper

            // compute the weight wi
            if(s_refined_last > 0){
                cv::Mat a = lambda * s_refined_last;
                cv::Mat b = phi * dtheta_last;
                cv::Mat c = zeta * biasa_last;
                cv::Mat d = ksi * pcb_refined_last;
                cv::Mat psi_pred = a+b+c+d;

                cv::Mat e = psi_pred - psi;
                double wi = std::exp(-cv::norm(e));
                if (cv::norm(e) > 0.05)   // 0.05  // can be deleted
                    wi = 0.0;


                lambda *= wi;
                phi *= wi;
                zeta *= wi;
                ksi *= wi;
            }

            // Stack into matrix C & D
            lambda.copyTo(C.rowRange(3*i+0,3*i+3).col(0));
            phi.colRange(0,2).copyTo(C.rowRange(3*i+0,3*i+3).colRange(1,3)); //only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
            zeta.copyTo(C.rowRange(3*i+0,3*i+3).colRange(3,6));
            ksi.copyTo(C.rowRange(3*i+0,3*i+3).colRange(6,9));
            psi.copyTo(D.rowRange(3*i+0,3*i+3));

        } // end for

        cv::Mat y;
        cv::solve(C,D,y,cv::DECOMP_LU|cv::DECOMP_NORMAL);
        // Debug log
        if(y.empty()){
            std::cerr << "[Warning] In refinement, cv::solve() result is empty" << std::endl;
            return false;
        }

        s_refined = y.at<float>(0);
        dthetaxy = y.rowRange(1,3);
        biasa_ = y.rowRange(3,6);
        pcb_refined = y.rowRange(6,9);


        // dtheta = [dx;dy;0]
        dthetaxy.copyTo(dtheta.rowRange(0,2));
        Eigen::Vector3d dthetaeig = Converter::toVector3d(dtheta);
        // Rwi_ = Rwi*Exp(dtheta)
        Eigen::Matrix3d Rwieig_ = RWIeig*Sophus::SO3::exp(dthetaeig).matrix();
        cv::Mat Rwi_ = Converter::toCvMat(Rwieig_); // Rwi_: after refined
        gw_refined = Rwi_*GI;

        // update last
        s_refined_last = s_refined;
        dtheta_last = dtheta;
        biasa_last = biasa_;
        pcb_refined_last = pcb_refined;

        // write p_bc_refined.txt and check convergence
        {
            cv::Mat pbc_refined = -Rcbstar.t()*pcb_refined;   // use estimated Rcbstar
            fp_bc_refined << mpCurrentKeyFrame->mTimeStamp<<" "
                           << pbc_refined.at<float>(0) << " " << pbc_refined.at<float>(1) << " " << pbc_refined.at<float>(2) << endl;

            fcalibrated_errors_pbc << mpCurrentKeyFrame->mTimeStamp<<" "
                                   << gt_x- pbc_refined.at<float>(0) << " " << gt_y-pbc_refined.at<float>(1) << " " << gt_z-pbc_refined.at<float>(2) << endl;

            // check whether the Pbc estimation has converge
            {
                // record estimated results and timestamps
                mvRecordEstimatedPbc.push_back(Converter::toVector3d(pbc_refined));
                mvRecordEstimatedPbcTimeStamps.push_back(mpCurrentKeyFrame->mTimeStamp);

                mbVINSInitPbcConverged = CheckPbcEstimationConverge();
            }
        }

    }   // End of Step 1.4: Accelerometer Bias Estimation, and Scale, Gravity Direction and Pcb Refinement


    // Debug log, write the result into txt
    {
        // cout<<"Time: "<<mpCurrentKeyFrame->mTimeStamp - mnStartTime<<", sstar: "<<sstar<<", s: "<<s_<<endl;

        fgw<<mpCurrentKeyFrame->mTimeStamp<<" "
           <<gw_refined.at<float>(0)<<" "<<gw_refined.at<float>(1)<<" "<<gw_refined.at<float>(2)<<" "
           <<gwstar.at<float>(0)<<" "<<gwstar.at<float>(1)<<" "<<gwstar.at<float>(2)<<" "<<endl;

        fscale<<mpCurrentKeyFrame->mTimeStamp<<" "
              <<s_refined<<" "<<sstar<<" "<<endl;
        fbiasa<<mpCurrentKeyFrame->mTimeStamp<<" "
              <<biasa_.at<float>(0)<<" "<<biasa_.at<float>(1)<<" "<<biasa_.at<float>(2)<<" "<<endl;

        fbiasg<<mpCurrentKeyFrame->mTimeStamp<<" "
              <<bgest(0)<<" "<<bgest(1)<<" "<<bgest(2)<<" "<<endl;
    }

    // ********************************
    bool bVIOInited = false;

    // if Rbc and Pbc are both convergent, then VIO has been successfully initialized.
    if(mbVINSInitRbcConverged && mbVINSInitPbcConverged)
    {
        bVIOInited = true;
        // Debug log
        std::cout << GREEN"Estimated Rbc and Pbc are both convergent." << RESET << std::endl;

//        // Debug log
//        for (size_t i=0; i<vErrors.size(); i++)
//        {
//            char str1[256]; sprintf(str1,"%lf", vTimestamps[i]);
//            cv::Mat e = vErrors[i];
//            double norm = vNorms[i];
//            double wi = vWi[i];
//            std::cout << "TimeStamp = "<<str1<<", "<<"e = " << e.t() << ", norm = " << norm << ", wi = " << wi  << endl;
//        }
    }

    // When failed. Or when you're debugging.
    // Reset biasg to zero, and re-compute imu-preintegrator.
    if(!bVIOInited)
    {
        ResetBiasgToZero(vScaleGravityKF);
    }
    else // successful initialization
    {
       // Set NavState , scale and bias for all KeyFrames
       // Scale
       double scale = s_refined;
       mnVINSInitScale = s_refined;
       // gravity vector in world frame
       mGravityVec = gw_refined;

       // mVINSInitRbc and mVINSInitPbc
       mVINSInitRbc = Rcbstar.t();
       mVINSInitPbc = -Rcbstar.t()*pcb_refined;

       // mVINSInitBiasg and mVINSInitBiasa
       Eigen::Vector3d biasa_eig = Converter::toVector3d(biasa_);
       Eigen::Vector3d gw_eig = Converter::toVector3d(gw_refined);

       mVINSInitBiasg = bgest;
       mVINSInitBiasa = biasa_eig;

       // mVINSInitTbc
       mVINSInitTbc = cv::Mat::eye(4,4,CV_32F);
       mVINSInitRbc.copyTo(mVINSInitTbc.rowRange(0,3).colRange(0,3));
       mVINSInitPbc.copyTo(mVINSInitTbc.rowRange(0,3).col(3));




       GetVINSInitTbc();

       for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
       {
           KeyFrame* pKF = *vit;
           // Position and rotation of visual SLAM
           cv::Mat wPc = pKF->GetPoseInverse().rowRange(0,3).col(3);                   // wPc
           cv::Mat Rwc = pKF->GetPoseInverse().rowRange(0,3).colRange(0,3);            // Rwc
           // Set position and rotation of navstate
           cv::Mat wPb = scale*wPc + Rwc*pcb_refined;
           pKF->SetNavStatePos(Converter::toVector3d(wPb));
           pKF->SetNavStateRot(Converter::toMatrix3d(Rwc*Rcbstar));
           // Update bias of Gyr & Acc
           pKF->SetNavStateBiasGyr(bgest);
           pKF->SetNavStateBiasAcc(biasa_eig);
           // Set delta_bias to zero. (only updated during optimization)
           pKF->SetNavStateDeltaBg(Eigen::Vector3d::Zero());
           pKF->SetNavStateDeltaBa(Eigen::Vector3d::Zero());
           /// Step 4. IV-D. Velocity Estimation
           // compute velocity
           if(pKF != vScaleGravityKF.back())
           {
               KeyFrame* pKFnext = pKF->GetNextKeyFrame();
               // IMU pre-int between pKF ~ pKFnext
               const IMUPreintegrator& imupreint = pKFnext->GetIMUPreInt();
               // Time from this(pKF) to next(pKFnext)
               double dt = imupreint.getDeltaTime();                                        // deltaTime
               cv::Mat dp = Converter::toCvMat(imupreint.getDeltaP());                      // deltaP
               cv::Mat Jpba = Converter::toCvMat(imupreint.getJPBiasa());                   // J_deltaP_biasa
               cv::Mat wPcnext = pKFnext->GetPoseInverse().rowRange(0,3).col(3);            // wPc next
               cv::Mat Rwcnext = pKFnext->GetPoseInverse().rowRange(0,3).colRange(0,3);     // Rwc next
               cv::Mat vel = - 1./dt*( scale*(wPc - wPcnext) + (Rwc - Rwcnext)*pcb_refined + Rwc*Rcbstar*(dp + Jpba*biasa_) + 0.5*gw_refined*dt*dt );
               Eigen::Vector3d vel_eig = Converter::toVector3d(vel);
               pKF->SetNavStateVel(vel_eig);
           }
           else
           {
               // If this is the last KeyFrame, no 'next' KeyFrame exists
               KeyFrame* pKFprev = pKF->GetPrevKeyFrame();
               const IMUPreintegrator& imupreint_prev_cur = pKF->GetIMUPreInt();
               double dt = imupreint_prev_cur.getDeltaTime();
               Eigen::Matrix3d Jvba = imupreint_prev_cur.getJVBiasa();
               Eigen::Vector3d dv = imupreint_prev_cur.getDeltaV();
               //
               Eigen::Vector3d velpre = pKFprev->GetNavState().Get_V();
               Eigen::Matrix3d rotpre = pKFprev->GetNavState().Get_RotMatrix();
               Eigen::Vector3d vel_eig = velpre + gw_eig*dt + rotpre*( dv + Jvba*biasa_eig );
               pKF->SetNavStateVel(vel_eig);
           }
       }

       // Re-compute IMU pre-integration at last.
       for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
       {
           KeyFrame* pKF = *vit;
           pKF->ComputePreInt();
       }
    }

    // Debug Log
    if(bVIOInited)
    {
        std::cout<<GREEN"\n[INFO] Spend Time: "<<mpCurrentKeyFrame->mTimeStamp - mnStartTime<<"s, sstar: "<<sstar<<", refined s: "<<s_refined<<std::endl;
        std::cout << "[INFO] VIO inited Success !" << RESET << std::endl;
    }

    return bVIOInited;

}


/**
 * @brief LocalMapping::TryInitVIO
 * @return
 * try to initial the Mono VIO
 */
bool LocalMapping::TryInitVIO(void)
{
    // only when the map has enough keyframes, can the system try to initialize
//    std::cout << "mpMap->KeyFramesInMap() = " << mpMap->KeyFramesInMap() << std::endl;
    if(mpMap->KeyFramesInMap()<=mnLocalWindowSize)
        return false;

    static bool fopened = false;
    static ofstream fgw, fscale, fbiasa, fcondnum, ftime, fbiasg;
    if(!fopened)
    {
        string tmpfilepath = mpParams->getTmpFilePath();
        fgw.open(tmpfilepath+"gw.txt");
        fscale.open(tmpfilepath+"scale.txt");
        fbiasa.open(tmpfilepath+"biasa.txt");
        fcondnum.open(tmpfilepath+"condum.txt");
        ftime.open(tmpfilepath+"computetime.txt");
        fbiasg.open(tmpfilepath+"biasg.txt");
        if(fgw.is_open() && fscale.is_open() && fbiasa.is_open() && fcondnum.is_open()
                && ftime.is_open() && fbiasg.is_open())
            fopened = true;
        else
        {
            std::cerr << "file open error in TryinitVIO" << std::endl;
            fopened = false;
        }

        fgw<<std::fixed<<std::setprecision(6);
        fscale<<std::fixed<<std::setprecision(6);
        fbiasa<<std::fixed<<std::setprecision(6);
        fcondnum<<std::fixed<<std::setprecision(6);
        ftime<<std::fixed<<std::setprecision(6);
        fbiasg<<std::fixed<<std::setprecision(6);
    }

    // TODO: Extrinsics between IMU and camera
    cv::Mat Tbc = mpParams->GetMatTbc();
    cv::Mat Rbc = Tbc.rowRange(0,3).colRange(0,3);
    cv::Mat pbc = Tbc.rowRange(0,3).col(3);
    cv::Mat Rcb = Rbc.t();
    cv::Mat pcb = -Rcb * pbc;

    // use all KeyFrames in map to compute
    vector<KeyFrame*> vScaleGravityKF = mpMap->GetAllKeyFrames();
    int N = vScaleGravityKF.size();

    /// Step 1. IV-A Gyroscope Bias Estimation
    // Try to compute initial gyro bias, using optimization with Gauss-Newtion
//    std::cout << "Trying to compute initial gyro bias ..." << std::endl;
    Vector3d bgest = Optimizer::OptimizeInitialGyroBias(vScaleGravityKF);
//    std::cout << "Step 1. IV-A Gyroscope Bias Estimation. The first bgest = " << bgest << std::endl;

    // Update biasg and pre-integration in LocalWindow. Remember to reset back to zero
    for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        pKF->SetNavStateBiasGyr(bgest);
    }
    for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        pKF->ComputePreInt();   /// 计算每个关键帧的预积分 (step 1. 已经计算出了biasg)
    }

    // Solve A*x=B for x=[s,gw] 4x1 vector
    cv::Mat A = cv::Mat::zeros(3*(N-2),4,CV_32F);
    cv::Mat B = cv::Mat::zeros(3*(N-2),1,CV_32F);
    cv::Mat I3 = cv::Mat::eye(3,3,CV_32F);

    /// Step 2.  IV-B. Scale and Gravity Approximation (no accelerometer bias)
    // Approx Scale and Gravity vector in 'world' frame (first KF's camera frame)
    for(int i=0; i<N-2; i++)
    {
        KeyFrame* pKF1 = vScaleGravityKF[i];
        KeyFrame* pKF2 = vScaleGravityKF[i+1];
        KeyFrame* pKF3 = vScaleGravityKF[i+2];
        // Delta time between frames
        double dt12 = pKF2->GetIMUPreInt().getDeltaTime();
        double dt23 = pKF3->GetIMUPreInt().getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(pKF3->GetIMUPreInt().getDeltaP());
        // Test log
        if(dt12!=pKF2->mTimeStamp-pKF1->mTimeStamp)
        {
            cerr<<"dt12 != pKF2->mTimeStamp-pKF1->mTimeStamp"<<endl;

            // display Id and display timestamps
            char str1[256], str2[256];
            sprintf(str1,"%lf", pKF1->mTimeStamp);
            sprintf(str2,"%lf", pKF2->mTimeStamp);
            std::cout << "pKF2->mTimeStamp = " << str2 << "  pKF1->mTimeStamp = " << str1 << " dt12 = " << dt12 << std::endl;
        }
        if(dt23!=pKF3->mTimeStamp-pKF2->mTimeStamp) cerr<<"dt23 != pKF3->mTimeStamp-pKF2->mTimeStamp"<<endl;

        // Pose of camera in world frame
        cv::Mat Twc1 = pKF1->GetPoseInverse();
        cv::Mat Twc2 = pKF2->GetPoseInverse();
        cv::Mat Twc3 = pKF3->GetPoseInverse();
        // Position of camera center
        cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
        cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
        cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
        // Rotation of camera, Rwc
        cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
        cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
        cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);

        // Stack to A/B matrix
        // lambda*s + beta*g = gamma
        cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
        cv::Mat beta = 0.5*I3*(dt12*dt12*dt23 + dt12*dt23*dt23);
        cv::Mat gamma = (Rc3-Rc2)*pcb*dt12 + (Rc1-Rc2)*pcb*dt23 + Rc1*Rcb*dp12*dt23 - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt12*dt23;
        lambda.copyTo(A.rowRange(3*i+0,3*i+3).col(0));
        beta.copyTo(A.rowRange(3*i+0,3*i+3).colRange(1,4));
        gamma.copyTo(B.rowRange(3*i+0,3*i+3));
        // Tested the formulation in paper, -gamma. Then the scale and gravity vector is -xx

        // Debug log
        //cout<<"iter "<<i<<endl;
    }

    double sstar;
    cv::Mat gwstar;

//    // Test cv::solve(), the result is the same as use cv::SVDecomp()
//    {
//        // Use svd to compute A*x=B, x=[s,gw] 4x1 vector
//        cv::Mat x;
//        cv::solve(A, B, x, cv::DECOMP_SVD);
//        sstar = x.at<float>(0);    // scale should be positive
//        gwstar = x.rowRange(1,4);   // gravity should be about ~9.8
//        cout<<"cv::solve(): scale sstar: "<<sstar<<endl;
//        cout<<"cv::solve(): gwstar: "<<gwstar.t()<< endl;
//    }

    // Use svd to compute A*x=B, x=[s,gw] 4x1 vector
    // A = u*w*vt, u*w*vt*x=B
    // Then x = vt'*winv*u'*B
    cv::Mat w,u,vt;
    // Note w is 4x1 vector by SVDecomp()
    // A is changed in SVDecomp() with cv::SVD::MODIFY_A for speed
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A);
    // Debug log
//    cout<<"u:"<<endl<<u<<endl;
//    cout<<"vt:"<<endl<<vt<<endl;
//    cout<<"w:"<<endl<<w<<endl;

    // Compute winv
    cv::Mat winv=cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<4;i++)
    {
        if(fabs(w.at<float>(i))<1e-10)
        {
            w.at<float>(i) += 1e-10;
            // Test log
            cerr<<"w(i) < 1e-10, w="<<endl<<w<<endl;
        }

        winv.at<float>(i,i) = 1./w.at<float>(i);
    }
    // Then x = vt'*winv*u'*B
    cv::Mat x = vt.t()*winv*u.t()*B;

    // x=[s,gw] 4x1 vector
    sstar = x.at<float>(0);    // scale should be positive
    gwstar = x.rowRange(1,4);   // gravity should be about ~9.8


    // Debug log
//    cout<<"scale sstar: "<<sstar<<endl;
//    cout<<"gwstar: "<<gwstar.t()<<", |gwstar|="<<cv::norm(gwstar) <<endl;

    // Test log
    if(w.type()!=I3.type() || u.type()!=I3.type() || vt.type()!=I3.type())
        cerr<<"different mat type, I3,w,u,vt: "<<I3.type()<<","<<w.type()<<","<<u.type()<<","<<vt.type()<<endl;

    /// Step 3. IV-C. Accelerometer Bias Estimation, and Scale and Gravity Direction Refinement
    // Use gravity magnitude 9.8 as constraint
    // gI = [0;0;1], the normalized gravity vector in an inertial frame, NED type with no orientation.
    cv::Mat gI = cv::Mat::zeros(3,1,CV_32F);
    gI.at<float>(2) = -1; //-1;    // wangjing: gI = [0;0;1], 但是论文 gI = [0;0;-1]
    // Normalized approx. gravity vecotr in world frame
    cv::Mat gwn = gwstar/cv::norm(gwstar);
    // Debug log
    // cout<<"gw normalized: "<<gwn<<endl;

    // vhat = (gI x gw) / |gI x gw|
    cv::Mat gIxgwn = gI.cross(gwn);
    double normgIxgwn = cv::norm(gIxgwn);
    cv::Mat vhat = gIxgwn/normgIxgwn;
    double theta = std::atan2(normgIxgwn,gI.dot(gwn));
    // Debug log
    // cout<<"vhat: "<<vhat<<", theta: "<<theta*180.0/M_PI<<endl;

    Eigen::Vector3d vhateig = Converter::toVector3d(vhat);
    Eigen::Matrix3d RWIeig = Sophus::SO3::exp(vhateig*theta).matrix();
    cv::Mat Rwi = Converter::toCvMat(RWIeig);
    cv::Mat GI = gI*ConfigParam::GetG();//9.8012;
    // Solve C*x=D for x=[s,dthetaxy,ba] (1+2+3)x1 vector
    cv::Mat C = cv::Mat::zeros(3*(N-2),6,CV_32F);
    cv::Mat D = cv::Mat::zeros(3*(N-2),1,CV_32F);

    for(int i=0; i<N-2; i++)
    {
        KeyFrame* pKF1 = vScaleGravityKF[i];
        KeyFrame* pKF2 = vScaleGravityKF[i+1];
        KeyFrame* pKF3 = vScaleGravityKF[i+2];
        // Delta time between frames
        double dt12 = pKF2->GetIMUPreInt().getDeltaTime();
        double dt23 = pKF3->GetIMUPreInt().getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(pKF3->GetIMUPreInt().getDeltaP());
        cv::Mat Jpba12 = Converter::toCvMat(pKF2->GetIMUPreInt().getJPBiasa());
        cv::Mat Jvba12 = Converter::toCvMat(pKF2->GetIMUPreInt().getJVBiasa());
        cv::Mat Jpba23 = Converter::toCvMat(pKF3->GetIMUPreInt().getJPBiasa());
        // Pose of camera in world frame
        cv::Mat Twc1 = pKF1->GetPoseInverse();
        cv::Mat Twc2 = pKF2->GetPoseInverse();
        cv::Mat Twc3 = pKF3->GetPoseInverse();
        // Position of camera center
        cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
        cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
        cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
        // Rotation of camera, Rwc
        cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
        cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
        cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);
        // Stack to C/D matrix
        // lambda*s + phi*dthetaxy + zeta*ba = psi
        cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
        cv::Mat phi = - 0.5*(dt12*dt12*dt23 + dt12*dt23*dt23)*Rwi*SkewSymmetricMatrix(GI);  // note: this has a '-', different to paper
        cv::Mat zeta = Rc2*Rcb*Jpba23*dt12 + Rc1*Rcb*Jvba12*dt12*dt23 - Rc1*Rcb*Jpba12*dt23;
        cv::Mat psi = (Rc1-Rc2)*pcb*dt23 + Rc1*Rcb*dp12*dt23 - (Rc2-Rc3)*pcb*dt12
                     - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt23*dt12 - 0.5*Rwi*GI*(dt12*dt12*dt23 + dt12*dt23*dt23); // note:  - paper
        lambda.copyTo(C.rowRange(3*i+0,3*i+3).col(0));
        phi.colRange(0,2).copyTo(C.rowRange(3*i+0,3*i+3).colRange(1,3)); //only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
        zeta.copyTo(C.rowRange(3*i+0,3*i+3).colRange(3,6));
        psi.copyTo(D.rowRange(3*i+0,3*i+3));

        // Debug log
        //cout<<"iter "<<i<<endl;
    }

    // Use svd to compute C*x=D, x=[s,dthetaxy,ba] 6x1 vector
    // C = u*w*vt, u*w*vt*x=D
    // Then x = vt'*winv*u'*D
    cv::Mat w2,u2,vt2;
    // Note w2 is 6x1 vector by SVDecomp()
    // C is changed in SVDecomp() with cv::SVD::MODIFY_A for speed
    cv::SVDecomp(C,w2,u2,vt2,cv::SVD::MODIFY_A);
    // Debug log
    //cout<<"u2:"<<endl<<u2<<endl;
    //cout<<"vt2:"<<endl<<vt2<<endl;
    //cout<<"w2:"<<endl<<w2<<endl;

    // Compute winv
    cv::Mat w2inv=cv::Mat::eye(6,6,CV_32F);
    for(int i=0;i<6;i++)
    {
        if(fabs(w2.at<float>(i))<1e-10)
        {
            w2.at<float>(i) += 1e-10;
            // Test log
            cerr<<"w2(i) < 1e-10, w="<<endl<<w2<<endl;
        }

        w2inv.at<float>(i,i) = 1./w2.at<float>(i);
    }
    // Then y = vt'*winv*u'*D
    cv::Mat y = vt2.t()*w2inv*u2.t()*D;

    double s_ = y.at<float>(0);
    cv::Mat dthetaxy = y.rowRange(1,3);
    cv::Mat biasa_ = y.rowRange(3,6);
    Vector3d biasa_eig = Converter::toVector3d(biasa_);

    // dtheta = [dx;dy;0]
    cv::Mat dtheta = cv::Mat::zeros(3,1,CV_32F);
    dthetaxy.copyTo(dtheta.rowRange(0,2));
    Eigen::Vector3d dthetaeig = Converter::toVector3d(dtheta);
    // Rwi_ = Rwi*exp(dtheta)
    Eigen::Matrix3d Rwieig_ = RWIeig*Sophus::SO3::exp(dthetaeig).matrix();
    cv::Mat Rwi_ = Converter::toCvMat(Rwieig_);


    // Debug log, write the result into txt
    {
        cv::Mat gwbefore = Rwi*GI;
        cv::Mat gwafter = Rwi_*GI;
//        cout<<"Time: "<<mpCurrentKeyFrame->mTimeStamp - mnStartTime<<", sstar: "<<sstar<<", s: "<<s_<<endl;

        fgw<<mpCurrentKeyFrame->mTimeStamp<<" "
           <<gwafter.at<float>(0)<<" "<<gwafter.at<float>(1)<<" "<<gwafter.at<float>(2)<<" "
           <<gwbefore.at<float>(0)<<" "<<gwbefore.at<float>(1)<<" "<<gwbefore.at<float>(2)<<" "
           <<endl;
        fscale<<mpCurrentKeyFrame->mTimeStamp<<" "
              <<s_<<" "<<sstar<<" "<<endl;
        fbiasa<<mpCurrentKeyFrame->mTimeStamp<<" "
              <<biasa_.at<float>(0)<<" "<<biasa_.at<float>(1)<<" "<<biasa_.at<float>(2)<<" "<<endl;
        fcondnum<<mpCurrentKeyFrame->mTimeStamp<<" "
                <<w2.at<float>(0)<<" "<<w2.at<float>(1)<<" "<<w2.at<float>(2)<<" "<<w2.at<float>(3)<<" "
                <<w2.at<float>(4)<<" "<<w2.at<float>(5)<<" "<<endl;
        //        ftime<<mpCurrentKeyFrame->mTimeStamp<<" "
        //             <<(t3-t0)/cv::getTickFrequency()*1000<<" "<<endl;
        fbiasg<<mpCurrentKeyFrame->mTimeStamp<<" "
              <<bgest(0)<<" "<<bgest(1)<<" "<<bgest(2)<<" "<<endl;
    }

    // ********************************
    // Todo:
    // Add some logic or strategy to confirm init status
    bool bVIOInited = false;
    if(mbFirstTry)
    {
       mbFirstTry = false;
       mnStartTime = mpCurrentKeyFrame->mTimeStamp;
    }

    /// 目前方案: VIO初始化执行15秒后强制设置成功
    if(mpCurrentKeyFrame->mTimeStamp - mnStartTime >= 15.0)
    {
       bVIOInited = true;
    }

    // When failed. Or when you're debugging.
    // Reset biasg to zero, and re-compute imu-preintegrator.
    if(!bVIOInited)
    {
       for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
       {
           KeyFrame* pKF = *vit;
           pKF->SetNavStateBiasGyr(Vector3d::Zero());
           pKF->SetNavStateBiasAcc(Vector3d::Zero());
           pKF->SetNavStateDeltaBg(Eigen::Vector3d::Zero());
           pKF->SetNavStateDeltaBa(Eigen::Vector3d::Zero());
       }
       for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
       {
           KeyFrame* pKF = *vit;
           pKF->ComputePreInt();
       }
    }
    else
    {
       // Set NavState , scale and bias for all KeyFrames
       // Scale
       double scale = s_;
       mnVINSInitScale = s_;
       // gravity vector in world frame
       cv::Mat gw = Rwi_*GI;
       mGravityVec = gw;
       Vector3d gweig = Converter::toVector3d(gw);

       for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
       {
           KeyFrame* pKF = *vit;
           // Position and rotation of visual SLAM
           cv::Mat wPc = pKF->GetPoseInverse().rowRange(0,3).col(3);                   // wPc
           cv::Mat Rwc = pKF->GetPoseInverse().rowRange(0,3).colRange(0,3);            // Rwc
           // Set position and rotation of navstate
           cv::Mat wPb = scale*wPc + Rwc*pcb;
           pKF->SetNavStatePos(Converter::toVector3d(wPb));
           pKF->SetNavStateRot(Converter::toMatrix3d(Rwc*Rcb));
           // Update bias of Gyr & Acc
           pKF->SetNavStateBiasGyr(bgest);
           pKF->SetNavStateBiasAcc(biasa_eig);
           // Set delta_bias to zero. (only updated during optimization)
           pKF->SetNavStateDeltaBg(Eigen::Vector3d::Zero());
           pKF->SetNavStateDeltaBa(Eigen::Vector3d::Zero());
           /// Step 4. IV-D. Velocity Estimation
           // compute velocity
           if(pKF != vScaleGravityKF.back())
           {
               KeyFrame* pKFnext = pKF->GetNextKeyFrame();
               // IMU pre-int between pKF ~ pKFnext
               const IMUPreintegrator& imupreint = pKFnext->GetIMUPreInt();
               // Time from this(pKF) to next(pKFnext)
               double dt = imupreint.getDeltaTime();                                       // deltaTime
               cv::Mat dp = Converter::toCvMat(imupreint.getDeltaP());       // deltaP
               cv::Mat Jpba = Converter::toCvMat(imupreint.getJPBiasa());    // J_deltaP_biasa
               cv::Mat wPcnext = pKFnext->GetPoseInverse().rowRange(0,3).col(3);           // wPc next
               cv::Mat Rwcnext = pKFnext->GetPoseInverse().rowRange(0,3).colRange(0,3);    // Rwc next

               cv::Mat vel = - 1./dt*( scale*(wPc - wPcnext) + (Rwc - Rwcnext)*pcb + Rwc*Rcb*(dp + Jpba*biasa_) + 0.5*gw*dt*dt );
               Eigen::Vector3d veleig = Converter::toVector3d(vel);
               pKF->SetNavStateVel(veleig);
           }
           else
           {
               // If this is the last KeyFrame, no 'next' KeyFrame exists
               KeyFrame* pKFprev = pKF->GetPrevKeyFrame();
               const IMUPreintegrator& imupreint_prev_cur = pKF->GetIMUPreInt();
               double dt = imupreint_prev_cur.getDeltaTime();
               Eigen::Matrix3d Jvba = imupreint_prev_cur.getJVBiasa();
               Eigen::Vector3d dv = imupreint_prev_cur.getDeltaV();
               //
               Eigen::Vector3d velpre = pKFprev->GetNavState().Get_V();
               Eigen::Matrix3d rotpre = pKFprev->GetNavState().Get_RotMatrix();
               Eigen::Vector3d veleig = velpre + gweig*dt + rotpre*( dv + Jvba*biasa_eig );
               pKF->SetNavStateVel(veleig);
           }
       }

       // Re-compute IMU pre-integration at last.
       for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
       {
           KeyFrame* pKF = *vit;
           pKF->ComputePreInt();
       }
    }

    // Debug Log
    if(bVIOInited)
    {
        std::cout<<"Time: "<<mpCurrentKeyFrame->mTimeStamp - mnStartTime<<", sstar: "<<sstar<<", s: "<<s_<<std::endl;
        std::cout << "[INFO] VIO inited Success !" << RESET << std::endl;
    }

    return bVIOInited;

}


bool LocalMapping::GetVINSInited()
{
    unique_lock<mutex> lock(mMutexVINSInitFlag);
    return mbVINSInited;
}

void LocalMapping::SetVINSInited(bool flag)
{
    unique_lock<mutex> lock(mMutexVINSInitFlag);
    mbVINSInited = flag;
}

bool LocalMapping::GetFirstVINSInited()
{
    unique_lock<mutex> lock(mMutexFirstVINSInitFlag);
    return mbFirstVINSInited;
}

void LocalMapping::SetFirstVINSInited(bool flag)
{
    unique_lock<mutex> lock(mMutexFirstVINSInitFlag);
    mbFirstVINSInited = flag;
}

bool LocalMapping::GetMapUpdateFlagForTracking()
{
    unique_lock<mutex> lock(mMutexMapUpdateFlag);
    return mbMapUpdateFlagForTracking;
}

void LocalMapping::SetMapUpdateFlagInTracking(bool flag)
{
    unique_lock<mutex> lock(mMutexMapUpdateFlag);
    mbMapUpdateFlagForTracking = flag;
//    if(!flag)
//        cout << "** 5--Set mbMapUpdateFlagForTracking = false" << endl;
    if(flag)
    {
        mpMapUpdateKF = mpCurrentKeyFrame;
//        std::cout << "*** mbMapUpdateFlagForTracking = true, mpMapUpdateKF->mnFrameId = " << mpMapUpdateKF->mnFrameId << std::endl;
    }
}

bool LocalMapping::GetEnableChangeMapUpdateFlagForTracking()
{
    unique_lock<mutex> lock(mMutexMapUpdateFlag);
    return mbEnableChangeMapUpdateFlagForTracking;
}

KeyFrame* LocalMapping::GetMapUpdateKF()
{
    unique_lock<mutex> lock(mMutexMapUpdateFlag);
    return mpMapUpdateKF;
}

void LocalMapping::KeyFrameCullingForMonoVI()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    KeyFrame* pOldestLocalKF = mlLocalKeyFrames.front();        /// for VI-ORB_SLAM
    KeyFrame* pPrevLocalKF = pOldestLocalKF->GetPrevKeyFrame(); /// for VI-ORB_SLAM
    KeyFrame* pNewestLocalKF = mlLocalKeyFrames.back();         /// for VI-ORB_SLAM
    // Test log
    if(pOldestLocalKF->isBad()) cerr<<"pOldestLocalKF is bad, check 1. id: "<<pOldestLocalKF->mnId<<endl;
    if(pPrevLocalKF) if(pPrevLocalKF->isBad()) cerr<<"pPrevLocalKF is bad, check 1. id: "<<pPrevLocalKF->mnId<<endl;
    if(pNewestLocalKF->isBad()) cerr<<"pNewestLocalKF is bad, check 1. id: "<<pNewestLocalKF->mnId<<endl;

    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;

        // Don't cull the oldest KF in LocalWindow,
        // And the KF before this KF
        if(pKF == pOldestLocalKF || pKF == pPrevLocalKF)
            continue;

        // Check time between Prev/Next Keyframe, if larger than 0.5s(for local)/3s(others), don't cull
        // Note, the KF just out of Local is similarly considered as Local
        KeyFrame* pPrevKF = pKF->GetPrevKeyFrame();
        KeyFrame* pNextKF = pKF->GetNextKeyFrame();
        if(pPrevKF && pNextKF)
        {
            double timegap=0.5;
            if(GetVINSInited())
                timegap = 3;

            // Test log
            if(pOldestLocalKF->isBad()) cerr<<"pOldestLocalKF is bad, check 1. id: "<<pOldestLocalKF->mnId<<endl;
            if(pPrevLocalKF) if(pPrevLocalKF->isBad()) cerr<<"pPrevLocalKF is bad, check 1. id: "<<pPrevLocalKF->mnId<<endl;
            if(pNewestLocalKF->isBad()) cerr<<"pNewestLocalKF is bad, check 1. id: "<<pNewestLocalKF->mnId<<endl;

            if(pKF->mnId >= pOldestLocalKF->mnId)
            {
                timegap = 0.1;    // third tested, good
                if(GetVINSInited())
                    timegap = 0.5;
                // Test log
                if(pKF->mnId >= pNewestLocalKF->mnId)
                    cerr<<"Want to cull Newer KF than LocalWindow? id/currentKFid:"<<pKF->mnId<<"/"<<mpCurrentKeyFrame->mnId<<endl;
            }
            if(fabs(pNextKF->mTimeStamp - pPrevKF->mTimeStamp) > timegap)
                continue;
        }


        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        if(nRedundantObservations>0.9*nMPs)
            pKF->SetBadFlag();
    }
}

double LocalMapping::GetVINSInitScale(void)
{
    return mnVINSInitScale;
}

cv::Mat LocalMapping::GetGravityVec()
{
    return mGravityVec;
}

cv::Mat LocalMapping::GetVINSInitTbc()
{
    // Debug log
    if(mVINSInitTbc.empty())
        std::cerr << RED"[Error] mVINSInitTbc is empty, why?" << RESET << std::endl;

    return mVINSInitTbc;
}

cv::Mat LocalMapping::GetVINSInitRbc()
{
    return mVINSInitRbc;
}

cv::Mat LocalMapping::GetVINSInitPbc()
{
    return mVINSInitPbc;
}


double LocalMapping::GetVariance(std::vector<double> &vector_)
{
    double sum = std::accumulate(std::begin(vector_), std::end(vector_), 0.0);
    double mean = sum / vector_.size();

    double accum = 0.0;
    std::for_each(std::begin(vector_), std::end(vector_), [&](const double d){
       accum += (d-mean)*(d-mean);
    });

    return accum / vector_.size();
}

double LocalMapping::GetStandardDeviation(std::vector<double> &vector_)
{
    return std::sqrt( GetVariance(vector_) );
}

double LocalMapping::GetVariance(std::vector<cv::Mat> &vector_)
{
    cv::Mat m = vector_[0];
    if( 1 != m.cols ){
        std::cerr << "[Warning] std::vector<cv::Mat> &vector_ has more than one cols." << std::endl;
        return -1.0;
    }

    cv::Mat sum = cv::Mat::zeros(m.rows, m.cols, CV_32F); // = std::accumulate(std::begin(vector_), std::end(vector_), cv::Mat::zeros(m.rows, m.cols));

    std::for_each(std::begin(vector_), std::end(vector_), [&](const cv::Mat d){
       sum += d;
    });

    cv::Mat mean = sum / vector_.size();

    double accum = 0.0;
    std::for_each(std::begin(vector_), std::end(vector_), [&](const cv::Mat d){
       cv::Mat mul = (d-mean).t() * (d-mean);
        accum += mul.at<float>(0,0);
    });

    return accum / vector_.size();
}

bool LocalMapping::CheckRbcEstimationConverge()
{
    static bool fopened = false;
    static ofstream fStandardDeviation;
    if(!fopened)
    {
        fStandardDeviation.open(mpParams->getTmpFilePath()+"StandardDeviationOfRbcEstimation.txt");
        if(fStandardDeviation.is_open()) fopened = true;
        else{
            std::cerr << "StandardDeviation.txt open faile" << std::endl;
            fopened = false;
        }

        fStandardDeviation<<std::fixed<<std::setprecision(6);
        fStandardDeviation << "timestamp staDev_yaw staDev_roll staDev_pitch" << endl;
    }


    std::vector<double> vYaw;
    std::vector<double> vRoll;
    std::vector<double> vPitch;
    for(size_t i=0; i<mvRecordEstimatedRbcTimeStamps.size(); i++)
    {
        // get the keyframes within 10 seconds
        if(mvRecordEstimatedRbcTimeStamps[i] > mpCurrentKeyFrame->mTimeStamp - 10){
            Eigen::Vector3d euler_bc = mvRecordEstimatedRbc[i];
            vYaw.push_back(euler_bc(0) * 180 / M_PI);
            vRoll.push_back(euler_bc(1) * 180 / M_PI);  // should be pitch
            vPitch.push_back(euler_bc(2) * 180 / M_PI); // should be roll
        }
    }

    // Ensure we have enough keyframes.
    // New keyframes may not be created when the aircraft hovers at one place. Once this happen,
    // the number of keyframes are rare within 10 seconds.
    if(vYaw.size()<10)  // keyframe should more than a threshold
        return false;

    double staDevOfYaw = GetStandardDeviation(vYaw);
    double staDevOfRoll = GetStandardDeviation(vRoll);
    double staDevOfPitch = GetStandardDeviation(vPitch);

    {   // Test log
//        char str1[256];
//        sprintf(str1,"%lf", mpCurrentKeyFrame->mTimeStamp-mvRecordEstimatedRbcTimeStamps[0]);
//        std::cout << "Estimated Rbc: time = " << str1
//                  << ",  staDevOfYaw = " << staDevOfYaw
//                  << ",  staDevOfRoll = " << staDevOfRoll
//                  << ",  staDevOfPitch = " << staDevOfPitch << std::endl;

        // write to .txt
        fStandardDeviation << mpCurrentKeyFrame->mTimeStamp<<" "
                           << staDevOfYaw << " " << staDevOfRoll << " " << staDevOfPitch << endl;
    }

//    if(staDevOfYaw<0.15 && staDevOfRoll<0.15 && staDevOfPitch<0.15) // standard deviation < threshold
    if(staDevOfYaw<0.1 && staDevOfRoll<0.1 && staDevOfPitch<0.1) // standard deviation < threshold
        return true;
    else
        return false;
}

bool LocalMapping::CheckPbcEstimationConverge()
{
    static bool fopened = false;
    static ofstream fStandardDeviation;
    if(!fopened)
    {
        fStandardDeviation.open(mpParams->getTmpFilePath()+"StandardDeviationOfPbcEstimation.txt");
        if(fStandardDeviation.is_open()) fopened = true;
        else{
            std::cerr << "StandardDeviationOfPbcEstimation.txt open faile" << std::endl;
            fopened = false;
        }

        fStandardDeviation<<std::fixed<<std::setprecision(6);
        fStandardDeviation << "timestamp staDev_x staDev_y staDev_z" << endl;
    }

    std::vector<double> vX;
    std::vector<double> vY;
    std::vector<double> vZ;
    for(size_t i=0; i<mvRecordEstimatedPbcTimeStamps.size(); i++)
    {
        // get the keyframes within 10 seconds
        if(mvRecordEstimatedPbcTimeStamps[i] > mpCurrentKeyFrame->mTimeStamp - 10){
            Eigen::Vector3d xyz = mvRecordEstimatedPbc[i];
            if(xyz(0)!=xyz(0) || xyz(1)!=xyz(1) || xyz(2)!=xyz(2))  // -NaN
                continue;

            vX.push_back(xyz(0));
            vY.push_back(xyz(1));
            vZ.push_back(xyz(2));
        }
    }

    // Ensure we have enough keyframes.
    // New keyframes may not be created when the aircraft hovers at one place. Once this happen,
    // the number of keyframes are rare within 10 seconds.
    if(vX.size()<10)  // keyframe should more than a threshold
        return false;

    double staDevOfX = GetStandardDeviation(vX);
    double staDevOfY = GetStandardDeviation(vY);
    double staDevOfZ = GetStandardDeviation(vZ);

    {   // Test log
//        char str1[256];
//        sprintf(str1,"%lf", mpCurrentKeyFrame->mTimeStamp-mvRecordEstimatedPbcTimeStamps[0]);
//        std::cout << "Estimated Pbc: time = " << str1
//                  << ",  staDevOfX = " << staDevOfX
//                  << ",  staDevOfY = " << staDevOfY
//                  << ",  staDevOfZ = " << staDevOfZ << std::endl;

        // write to .txt
        fStandardDeviation << mpCurrentKeyFrame->mTimeStamp<<" "
                           << staDevOfX << " " << staDevOfY << " " << staDevOfZ << endl;
    }

    if(staDevOfX<0.02 && staDevOfY<0.02 && staDevOfZ<0.02) // standard deviation < threshold
//    if(staDevOfX<0.01 && staDevOfY<0.01 && staDevOfZ<0.01) // standard deviation < threshold
        return true;
    else
        return false;
}

void LocalMapping::ResetBiasgToZero(std::vector<KeyFrame*> vScaleGravityKF)
{
    for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        pKF->SetNavStateBiasGyr(Vector3d::Zero());
        pKF->SetNavStateBiasAcc(Vector3d::Zero());
        pKF->SetNavStateDeltaBg(Eigen::Vector3d::Zero());
        pKF->SetNavStateDeltaBa(Eigen::Vector3d::Zero());
    }
    for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        pKF->ComputePreInt();
    }
}

/********************************************************************************/
/***************************** for VI-ORB_SLAM2 End *****************************/
/********************************************************************************/


LocalMapping::LocalMapping(Map *pMap, const float bMonocular):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
    mbMonoVIEnable = false;
    mbDeactiveLoopCloserInMonoVI = false;

    mnStartTime = 0.0;
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{

    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            bCreateNewMapPointsFinished = false;

//            std::cout << "[INFO] NewMapPointsCreatedByLastKF = " << nNewMapPointsCreatedByLastKF << std::endl;
            // BoW conversion and insertion in Map
            Timer timerOfProcessNewKeyFrame;
            ProcessNewKeyFrame();
            mTimeOfProcessNewKeyFrame = timerOfProcessNewKeyFrame.runTime_ms();
//            std::cout << " -- 1 -- mTimeOfProcessNewKeyFrame = " << mTimeOfProcessNewKeyFrame << std::endl;

            // Check recent MapPoints
            Timer timerOfMapPointCulling;
            MapPointCulling();
            mTimeOfMapPointCulling = timerOfMapPointCulling.runTime_ms();
//            std::cout << " -- 2 -- mTimeOfMapPointCulling = " << mTimeOfMapPointCulling << std::endl;

            // Triangulate new MapPoints
            Timer timerOfCreateNewMapPoints;
            CreateNewMapPoints();

            mTimeOfCreateNewMapPoints = timerOfCreateNewMapPoints.runTime_ms();
//            std::cout << " -- 3 -- mTimeOfCreateNewMapPoints = " << mTimeOfCreateNewMapPoints << std::endl;

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                Timer timerOfSearchInNeighbors;
                SearchInNeighbors();
                mTimeOfSearchInNeighbors = timerOfSearchInNeighbors.runTime_ms();
//                std::cout << " -- 4 -- mTimeOfSearchInNeighbors = " << mTimeOfSearchInNeighbors << std::endl;
            }
            bCreateNewMapPointsFinished = true;

            mbAbortBA = false;

            // 已经处理完队列中的最后的一个关键帧，并且闭环检测没有请求停止LocalMapping
            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // TODO: Disable the change of mbMapUpdateFlagForTracking before LocalBundleAdjustment
                mbEnableChangeMapUpdateFlagForTracking = false;

                // Local BA
                Timer timerOfLocalBA;
                if(mpMap->KeyFramesInMap()>2)
                {
                    // Debug log
                    if(mbAbortBA) std::cout << MAGENTA"[INFO] *** Before Optimizer::LocalBundleAdjustment, mbAbortBA is set to true. "
                                                      "mpCurrentKeyFrame->mnFrameId = " << mpCurrentKeyFrame->mnFrameId << RESET << std::endl;

                    if( !mbMonoVIEnable || !GetVINSInited())
                        Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap, this);
                    else    /// for VI-ORB_SLAM, if VIO has been initialized, then perform Local BA with IMU in a localwindow
                    {
                        if(mpParams->GetEstimateExtrinsicBetweenCameraAndIMU())
                            Optimizer::LocalBundleAdjustmentNavState(mpCurrentKeyFrame,mlLocalKeyFrames,&mbAbortBA, mpMap, mGravityVec, GetVINSInitTbc(), this);
                        else
                            Optimizer::LocalBundleAdjustmentNavState(mpCurrentKeyFrame,mlLocalKeyFrames,&mbAbortBA, mpMap, mGravityVec, mpParams->GetMatTbc(), this);
                    }

                    // Debug log
                    if(mbAbortBA) std::cout << MAGENTA"[INFO] *** After Optimizer::LocalBundleAdjustment, mbAbortBA is set to true. "
                                                      "mpCurrentKeyFrame->mnFrameId = " << mpCurrentKeyFrame->mnFrameId << RESET << std::endl;
                }
                mTimeOfLocalBA = timerOfLocalBA.runTime_ms();
//                std::cout << " -- 5 -- mTimeOfLocalBA = " << mTimeOfLocalBA << std::endl;

                /// for VI-ORB_SLAM, Try to initialize VIO, if not inited
                if(mbMonoVIEnable)
                {
                    if(!GetVINSInited())    // if not inited
//                    static bool vinsinited = false;
//                    if(!vinsinited)
                    {
                        // Processing time
                        static bool fopened = false;
                        static ofstream fProcessing_Time;
                        if(!fopened)
                        {
                            fProcessing_Time.open(mpParams->getTmpFilePath()+"Processing_Time.txt");
                            if(fProcessing_Time.is_open())
                                fopened = true;

                            fProcessing_Time<<std::fixed<<std::setprecision(6);
                            fProcessing_Time << "timestample processing_time_ms" << endl;
                        }

                        Timer timerOffProcessing_Time;
                        bool bVIOInited = false;
                        if(mpParams->GetEstimateExtrinsicBetweenCameraAndIMU())
                            bVIOInited = TryInitVIOWithoutPreCalibration();    // estimate Rbc and Pbc as well as scale, gravity, biasg and biasa
                        else
                            bVIOInited = TryInitVIO(); // use the pre-estimated Tbc, estimate scale, gravity, biasg and biasa

                        fProcessing_Time << mpCurrentKeyFrame->mTimeStamp<<" "
                                         << timerOffProcessing_Time.runTime_ms() << endl;

                        SetVINSInited(bVIOInited);
                        if(bVIOInited)
                        {
//                            vinsinited = true;

                            // TODO: Update map scale and set initialization flag
                            // Update map scale
                            mpMap->UpdateScale(mnVINSInitScale);
                            // Set initialization flag
                            SetFirstVINSInited(true);

                            std::cout << GREEN"[INFO] VIO inited. mpCurrentKeyFrame.mnId = " << mpCurrentKeyFrame->mnId
                                      << "  mnFrameId = " << mpCurrentKeyFrame->mnFrameId << RESET << std::endl;

                        }
                    }
                }

                // Check redundant local Keyframes
                Timer timerOfKeyFrameCulling;
                if(mbMonoVIEnable)
                    KeyFrameCullingForMonoVI();
                else
                    KeyFrameCulling();
                mTimeOfKeyFrameCulling = timerOfKeyFrameCulling.runTime_ms();
//                std::cout << " -- 6 -- mTimeOfKeyFrameCulling = " << mTimeOfKeyFrameCulling << std::endl;

                // TODO: Enable the change of mbMapUpdateFlagForTracking after LocalBundleAdjustment
                mbEnableChangeMapUpdateFlagForTracking = true;
            }

            /// LoopCloser detect loop closure for each new KeyFrame.
            /// if we comment this line, then the LoopCloser would not receive new KeyFrame,
            /// then LoopCloser would not work.
            /// Deactive LoopCloser for VI-ORB_SLAM -- 20170626
            if(!mbDeactiveLoopCloserInMonoVI)
                mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);

        }
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::AssociateMapPointsForMultiThread(
        const vector<MapPoint*> vpMapPointMatches,
        size_t &i_count)
{
    size_t iend = vpMapPointMatches.size();

    while(i_count<iend)
    {
        size_t i = 0;
        {
            unique_lock<mutex> lock(mMutexAssociateMapPointConutIncrease);
            i = i_count;
            i_count ++;
        }

        if(i_count>=iend) return;

        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            // 非当前帧生成的MapPoints
            // 为当前帧在Tracking::TrackLocalMap()过程跟踪到的MapPoints更新属性
            if(!pMP->isBad())
            {
                // 检测pMP是否已经与当前KF建立联系. 对新插入的关键帧,一般还未建立联系
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    // 我的理解
                    // 此部分的MapPoint来自于Tracking线程中的TrackLocalMap()里 SearchLocalPoints()函数,
                    // 该函数将mvpLocalMapPoints投影到当前帧，并且为当前帧中的特征点增加对应的MapPoint (注意:并未对MapPoint增加观测)
                    pMP->AddObservation(mpCurrentKeyFrame, i);  // 添加观测. i: pMP在KF->mvpMapPoints的index
                    pMP->UpdateNormalAndDepth();                // 更新该点的平均观测方向和观测距离范围
                    pMP->ComputeDistinctiveDescriptors();       // 加入关键帧后,更新3D点的最佳描述子
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    // 我的理解：　对Monocular, 此部分的MapPoint来自于Tracking线程中的TrackWithMotionModel()

                    // 当前帧生成的MapPoints
                    // 将双目或RGBD跟踪过程中新插入的MapPoints放入mlpRecentAddedMapPoints，等待检查
                    // CreateNewMapPoints 函数中通过三角化也会生成MapPoints
                    // 这些MapPoints都会经过MapPointCulling函数的检验
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }
}

/**
 * @brief 处理列表中的第一个关键帧
 *
 * - 计算Bow，加速三角化新的MapPoints
 * - 关联当前关键帧至MapPoints，并更新MapPoints的平均观测方向和观测距离范围
 * - 插入关键帧，更新Covisibility图和Essential图
 * @see VI-A keyframe insertion
 */
void LocalMapping::ProcessNewKeyFrame()
{
    // 步骤1：从缓冲队列中取出一帧关键帧
    // Tracking线程向LocalMapping中插入关键帧存在该队列中
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        // 从列表中获得一个等待被插入的关键帧
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        // 步骤2: 计算该关键帧特征点的BoW映射关系
        mlNewKeyFrames.pop_front();
    }

    Timer timerOfComputeBow;
    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    mTimeOfComputeBow = timerOfComputeBow.runTime_ms();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    // 步骤3: 将Tracking::TrackLocalMap()中新匹配上的MapPoints与新关键帧进行关联
    // 在Tracking::TrackLocalMap()函数中将局部地图中的MapPoints与当前帧进行了匹配 ( SearchLocalPoints() ),
    // 但是没有对这些匹配上的MapPoints与当前关键帧进行关联
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    Timer timerOfAssociateMapPoints;

    /// multi threads
    {
        size_t i_count = 0;
        //AssociateMapPointsForMultiThread(vpMapPointMatches, std::ref(i_count));

        std::thread thread1([&] { AssociateMapPointsForMultiThread(vpMapPointMatches, std::ref(i_count)); } );
        std::thread thread2([&] { AssociateMapPointsForMultiThread(vpMapPointMatches, std::ref(i_count)); } );
        std::thread thread3([&] { AssociateMapPointsForMultiThread(vpMapPointMatches, std::ref(i_count)); } );
        AssociateMapPointsForMultiThread(vpMapPointMatches, std::ref(i_count));
        thread1.join();
        thread2.join();
        thread3.join();

        size_t vpMapPointMatches_size = vpMapPointMatches.size();
        while(i_count<vpMapPointMatches_size) {}
    }


//    /// normal, without multi thread
//    {
//        for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
//        {
//            MapPoint* pMP = vpMapPointMatches[i];
//            if(pMP)
//            {
//                // 非当前帧生成的MapPoints
//                // 为当前帧在Tracking::TrackLocalMap()过程跟踪到的MapPoints更新属性
//                if(!pMP->isBad())
//                {
//                    // 检测pMP是否已经与当前KF建立联系. 对新插入的关键帧,一般还未建立联系
//                    if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
//                    {
//                        // 我的理解
//                        // 此部分的MapPoint来自于Tracking线程中的TrackLocalMap()里 SearchLocalPoints()函数,
//                        // 该函数将mvpLocalMapPoints投影到当前帧，并且为当前帧中的特征点增加对应的MapPoint (注意:并未对MapPoint增加观测)
//                        pMP->AddObservation(mpCurrentKeyFrame, i);  // 添加观测. i: pMP在KF->mvpMapPoints的index
//                        pMP->UpdateNormalAndDepth();                // 更新该点的平均观测方向和观测距离范围
//                        pMP->ComputeDistinctiveDescriptors();       // 加入关键帧后,更新3D点的最佳描述子
//                    }
//                    else // this can only happen for new stereo points inserted by the Tracking
//                    {
//                        // 我的理解：　对Monocular, 此部分的MapPoint来自于Tracking线程中的TrackWithMotionModel()

//                        // 当前帧生成的MapPoints
//                        // 将双目或RGBD跟踪过程中新插入的MapPoints放入mlpRecentAddedMapPoints，等待检查
//                        // CreateNewMapPoints 函数中通过三角化也会生成MapPoints
//                        // 这些MapPoints都会经过MapPointCulling函数的检验
//                        mlpRecentAddedMapPoints.push_back(pMP);
//                    }
//                }
//            }
//        }
//    }

    mTimeOfAssociateMapPoints = timerOfAssociateMapPoints.runTime_ms();

    // Update links in the Covisibility Graph
    // 步骤4：更新关键帧间的连接关系，Covisibility图和Essential图(tree)
    Timer timerOfUpdateConnections;
    mpCurrentKeyFrame->UpdateConnections();

    mTimeOfUpdateConnections = timerOfUpdateConnections.runTime_ms();

    /// for VI-ORB_SLAM
    if(mbMonoVIEnable)
    {
        // Delete bad KF in LocalWindow /// for VI-ORB_SLAM
        DeleteBadInLocalWindow();
        // Add Keyframe to LocalWindow  /// for VI-ORB_SLAM
        AddToLocalWindow(mpCurrentKeyFrame);
    }

    // Insert Keyframe in Map
    // 步骤5: 将该关键帧插入到地图中
    mpMap->AddKeyFrame(mpCurrentKeyFrame);

}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    // 遍历等待检查的MapPoints
    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            // 步骤1：已经是坏点的MapPoints直接从检查链表中删除
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            // 步骤2：将不满足VI-B条件的MapPoint剔除
            // VI-B 条件1：
            // 跟踪到该MapPoint的Frame数相比预计可观测到该MapPoint的Frame数的比例需大于25%
            // IncreaseFound / IncreaseVisible < 25%，注意不一定是关键帧。
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            // 步骤3：将不满足VI-B条件的MapPoint剔除
            // VI-B 条件2：从该点建立开始，到现在已经过了不小于2个关键帧
            // 但是观测到该点的关键帧数却不超过cnThObs帧，那么该点检验不合格
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);   // 超过3个关键帧,该点还没正式插入Map,则删除该MapPoint
        else
            lit++;
    }
}

/**
 *  相机运动过程中和共视程度比较高的关键帧通过三角化恢复出一些MapPoints
 */
void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    if(mbMonocular)
        nn=20;
    // 步骤1: 在当前关键帧的共视关键帧中找到共视程度最高的nn帧相邻帧
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));

    // 得到当前关键帧在世界坐标系中的坐标
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    int nnew=0;
    nNewMapPointsCreatedByLastKF = 0;

    // Search matches with epipolar restriction and triangulate
    // 步骤2: 遍历相邻关键帧vpNeighKFs
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())  // 一旦有新关键帧插入,立刻跳出该函数,处理新的关键帧
        {
            std::cout << BLUE"[INFO] Receive new KeyFrame when processing CreateNewMapPoints(). Exit to process new KeyFrame." << RESET << std::endl;
            return;
        }

        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();      // 相邻关键帧在世界坐标系中的坐标
        cv::Mat vBaseline = Ow2-Ow1;                // 基线向量,两个关键帧间的相机位移
        const float baseline = cv::norm(vBaseline); // 基线长度

        // 步骤3: 判断相机运动的基线是不是足够长
        if(!mbMonocular)
        {
            // 如果是立体相机,关键帧的间距太小时不能生成3D点
            if(baseline<pKF2->mb)
            continue;
        }
        else
        {
            // 邻接关键帧的场景深度中值
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            // baseline与景深的比例
            const float ratioBaselineDepth = baseline/medianDepthKF2;
            // 如果特别远(比例特别小),那么不考虑当前邻接的关键帧,不生成3D点
            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        // 步骤4: 根据两个关键帧的位姿计算他们之间的基本矩阵
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        // 步骤5: 通过极线约束限制匹配时的搜索范围,进行特征点匹配
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        // 步骤6：对每对匹配通过三角化生成3D点,和Triangulate函数差不多
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            // 步骤6.1：取出匹配特征点

            // 当前匹配对在当前关键帧中的索引
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            // 当前匹配在当前关键帧中的特征点
            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            // mvuRight中存放着双目的深度值，如果不是双目，其值将为-1
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            // 当前匹配在邻接关键帧中的特征点
            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            // mvuRight中存放着双目的深度值，如果不是双目，其值将为-1
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            // 步骤6.2：利用匹配点反投影得到视差角
            // 特征点反投影
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            // 由相机坐标系转到世界坐标系，得到视差角余弦值
            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            // 加1是为了让cosParallaxStereo随便初始化为一个很大的值
            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            // 步骤6.3：对于双目，利用双目得到视差角
            if(bStereo1)        // 双目,且有深度
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)   // 单目,且有深度
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            // 得到双目观测的视差角
            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            // 步骤6.4：三角化恢复3D点
            cv::Mat x3D;
            // cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998)表明视差角正常
            // cosParallaxRays<cosParallaxStereo表明视差角很小
            // 视差角度小时用三角法恢复3D点，视差角大时用双目恢复3D点（双目以及深度有效）
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);                
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
                continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                continue;*/
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            // 步骤6.8：三角化生成3D点成功，构造成MapPoint
            MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);

            // 步骤6.9：为该MapPoint添加属性：
            // a.观测到该MapPoint的关键帧
            // b.该MapPoint的描述子
            // c.该MapPoint的平均观测方向和深度范围
            pMP->AddObservation(mpCurrentKeyFrame,idx1);            
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            // 步骤6.8：将新产生的点放入检测队列
            // 这些MapPoints都会经过MapPointCulling函数的检验
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;

            nNewMapPointsCreatedByLastKF ++;
        }
    }
}

/**
 * 检查并融合当前关键帧与相邻帧（两级相邻）重复的MapPoints
 */
void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }  

        if(nRedundantObservations>0.9*nMPs)
            pKF->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);

    if(mbResetRequested)
    {
        std::cout << "LocalMapping::ResetIfRequested()" << std::endl;
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;

        if(mbMonoVIEnable)
        {
//            cout << "mlLocalKeyFrames.size() = " << mlLocalKeyFrames.size() << endl;
            mlLocalKeyFrames.clear();
            // Add resetting init flags
            SetVINSInited(false);
            mbFirstTry = true;

//            // clear the record
//            mvRecordEstimatedPbc.clear();
//            mvRecordEstimatedRbcTimeStamps.clear();
//            mvRecordEstimatedPbc.clear();
//            mvRecordEstimatedPbcTimeStamps.clear();



        }
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
