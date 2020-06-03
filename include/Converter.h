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

#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include "../src/IMU/imudata.h"
#include "../src/IMU/IMUPreintegrator.h"
#include "../src/IMU/NavState.h"

namespace ORB_SLAM2
{

class Converter
{
/// for VI-ORB_SLAM2
/********************************************************************************/
/**************************** for VI-ORB_SLAM2 Start ****************************/
/********************************************************************************/
public:
    static void updateNS(NavState& ns, const IMUPreintegrator& imupreint, const Vector3d& gw);
    static void updateNS(NavState& ns, const IMUPreintegrator& imupreint, const Vector3d& gw, const Eigen::Vector3d& dbiasg, const Eigen::Vector3d& dbiasa);
    static cv::Mat toCvMatInverse(const cv::Mat &T12);

    static std::vector<cv::Mat> toDescriptorVector(
            const cv::Mat &Descriptors,
            int total_parts,
            int part_index);

/********************************************************************************/
/***************************** for VI-ORB_SLAM2 End *****************************/
/********************************************************************************/

public:
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);


    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);
    static cv::Mat toSkew(const Eigen::Vector3d v);
    static cv::Mat toSkew(const cv::Mat &v);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
    static Eigen::Matrix4d toMatrix4d(const cv::Mat &cvMat4);

    static std::vector<float> toQuaternion(const cv::Mat &M);

    static Eigen::Vector3d toEulerAngles(const cv::Mat &cvMat3);    // ZYX: yaw, pitch, roll

};

}// namespace ORB_SLAM

#endif // CONVERTER_H
