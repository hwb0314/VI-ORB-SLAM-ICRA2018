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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

#include "MsgSync/MsgSynchronizer.h"
#include "../../../src/IMU/imudata.h"
#include "../../../src/IMU/configparam.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

typedef sensor_msgs::ImuConstPtr    SensorMsgImuPtr;// ImuMsgPtr;
typedef sensor_msgs::Imu            SensorMsgImu;
typedef sensor_msgs::ImageConstPtr  SensorMsgImagePtr;
typedef sensor_msgs::Image          SensorMsgImage;

using namespace std;

void SaveTimestamps(const vector<double> vTimestamps, const string &filename);

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg, const std::vector<ORB_SLAM2::IMUData> vimuData);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MONOCULAR_VIO");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_VIO ORB_VIO path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }    

    // -----------------------------------------------------------
    /// Step 1: Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ORB_SLAM2::ConfigParam config(argv[2]);
    ORB_SLAM2::ConfigParam* pParams = new ORB_SLAM2::ConfigParam(argv[2]);

    SLAM.SetConfigParam(pParams);

//    if(config.GetRunningMode() == 1)
//        SLAM.SetMonoVIEnable(true);   // set mbMonoVI=true, use MonoVI,
//    if(config.GetDeactiveLoopClosure() == true)
//        SLAM.SetDeactiveLoopCloserInMonoVI(true);   // set deactive loop closure detection

    if(pParams->GetRunningMode() == 1)
        SLAM.SetMonoVIEnable(true);   // set mbMonoVI=true, use MonoVI,
    if(pParams->GetDeactiveLoopClosure() == true)
        SLAM.SetDeactiveLoopCloserInMonoVI(true);   // set deactive loop closure detection


    ImageGrabber igb(&SLAM);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vector<double> vImageTimestamps;
    vector<double> vImuTimestamps;

    vector<double> vTimesOfProcessingFrame;
    vector<double> vTimesOfTrack;
    vector<double> vTimesOfConstructFrame;
    vector<double> vTimesOfTrackWithIMU;
    vector<double> vTimesOfTrackLocalMapWithIMU;

    vector<double> vTimesOfComputePyramid;
    vector<double> vTimesOfComputeKeyPointsOctTree;
    vector<double> vTimesOfComputeDescriptor;

    /**
     * @brief added data sync
     */
//    double imageMsgDelaySec = config.GetImageDelayToIMU();
    double imageMsgDelaySec = pParams->GetImageDelayToIMU();
    ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
    /// Step 2: Subscribing the image and imu topics, as well as recalling the response function to record data
    ros::NodeHandle nh;
    ros::Subscriber imagesub = nh.subscribe("/cam0/image_raw", 3, &ORBVIO::MsgSynchronizer::imageCallback, &msgsync);
    ros::Subscriber imusub = nh.subscribe("/imu0", 300, &ORBVIO::MsgSynchronizer::imuCallback, &msgsync);

    SensorMsgImagePtr imageMsg;
    std::vector<SensorMsgImuPtr> vimuMsg;

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
//    const bool bAccMultiply98 = config.GetAccMultiply9p8();     // default: 0
    const bool bAccMultiply98 = pParams->GetAccMultiply9p8();     // default: 0

    ros::Rate r(1000);
    Timer tTimeOfNoImage;
    bool bCheckTimeOfNoImage = false;
    int nImages = 0;    // image number
    while(ros::ok())
    {
        bool bdata = msgsync.getRecentMsgs(imageMsg, vimuMsg);  // get image and its all imu datas collected from last image
        if (bdata)  // subscribe a new image
        {
            nImages ++;

            // reconstruct the std::vector<SensorMsgImuPtr> into std::vector<ORB_SLAM2::IMUData>
            std::vector<ORB_SLAM2::IMUData> vimuData;
            for (size_t i=0, iend=vimuMsg.size(); i<iend; i++)
            {
                SensorMsgImuPtr imuMsg = vimuMsg[i];
                double wx = imuMsg->angular_velocity.x;
                double wy = imuMsg->angular_velocity.y;
                double wz = imuMsg->angular_velocity.z;
                double ax = imuMsg->linear_acceleration.x;
                double ay = imuMsg->linear_acceleration.y;
                double az = imuMsg->linear_acceleration.z;
                double timu = imuMsg->header.stamp.toSec();
                vImuTimestamps.push_back(timu);
                if(bAccMultiply98)
                {
                    ax *= g3dm;
                    ay *= g3dm;
                    az *= g3dm;
                }
                ORB_SLAM2::IMUData imudata( wx, wy, wz, ax, ay, az, timu );
                vimuData.push_back( imudata );
            }



            double tframe = imageMsg->header.stamp.toSec();
            vImageTimestamps.push_back( tframe );    // record stamps of new image

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            /// grab image and track
            igb.GrabImage(imageMsg, vimuData);

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000; // ms

            // Time statistic: Only record the time of successful tracking frame using vision & IMU.
            //                 Ignore the time-consuming of VI-ORB_SLAM initialization section since it uses only vision.
            if(SLAM.GetTrackWithIMUState())
            {
//                std::cout << "record time " << std::endl;
                vTimesTrack.push_back(ttrack);      // record track time of new image

                vTimesOfProcessingFrame.push_back(SLAM.GetTimeOfProcessingFrame());
                vTimesOfTrack.push_back(SLAM.GetTimeOfTrack());
                vTimesOfConstructFrame.push_back(SLAM.GetTimeOfConstructFrame());
                vTimesOfTrackWithIMU.push_back(SLAM.GetTimeOfTrackWithIMU());
                vTimesOfTrackLocalMapWithIMU.push_back(SLAM.GetTimeOfTrackLocalMapWithIMU());

                vTimesOfComputePyramid.push_back(SLAM.GetTimeOfComputePyramid());
                vTimesOfComputeKeyPointsOctTree.push_back(SLAM.GetTimeOfComputeKeyPointsOctTree());
                vTimesOfComputeDescriptor.push_back(SLAM.GetTImeOfComputeDescriptor());
            }

            bCheckTimeOfNoImage = true;
            tTimeOfNoImage.freshTimer(); // refresh timer
        }

        // Stop system: if more than 10s the system do not receive any image
        if( tTimeOfNoImage.runTime_s() > 10 && bCheckTimeOfNoImage )
        {
//            // Stop all threads
//            SLAM.Shutdown();

            double totaltime = 0;

            double totalTimeOfProcessingFrame = 0;
            double totalTimeOfTrack = 0;
            double totalTimeOfConstructFrame = 0;
            double totalTimeOfTrackWithIMU = 0;
            double totalTimeOfTrackLocalMapWithIMU = 0;

            double totalTimeOfComputePyramid = 0;
            double totalTimeOfComputeKeyPointsOctTree = 0;
            double totalTimeOfComputeDescriptor = 0;

            int timeRecordNum = vTimesOfTrack.size();

            for(int ni=0; ni<timeRecordNum; ni++) // nImages
            {
                totaltime+=vTimesTrack[ni];

                totalTimeOfProcessingFrame += vTimesOfProcessingFrame[ni];
                totalTimeOfTrack += vTimesOfTrack[ni];
                totalTimeOfConstructFrame += vTimesOfConstructFrame[ni];
                totalTimeOfTrackWithIMU += vTimesOfTrackWithIMU[ni];
                totalTimeOfTrackLocalMapWithIMU += vTimesOfTrackLocalMapWithIMU[ni];

                totalTimeOfComputePyramid += vTimesOfComputePyramid[ni];
                totalTimeOfComputeKeyPointsOctTree += vTimesOfComputeKeyPointsOctTree[ni];
                totalTimeOfComputeDescriptor += vTimesOfComputeDescriptor[ni];

            }
            cout << endl << "-------" << endl;
            cout << "Time statistic: Only record the time of successful tracking frame using vision & IMU." << endl;
            cout << "                Ignore the time-consuming of VI-ORB_SLAM initialization section since it uses only vision." << endl;
//            cout << "median tracking time: " << vTimesTrack[timeRecordNum/2] << " ms"<< endl;
//            cout << "mean tracking time: " << totaltime/timeRecordNum << " ms" << endl;

            cout << "| mean time of ProcessingFrame:              " << totalTimeOfProcessingFrame           / timeRecordNum << " ms" << endl;
            cout << "   | mean time of ConstructFrame:              " << totalTimeOfConstructFrame            / timeRecordNum << " ms" << endl;
            cout << "       | mean time of ComputePyramid:              " << totalTimeOfComputePyramid            / timeRecordNum << " ms" << endl;
            cout << "       | mean time of ComputeKeyPointsOctTre:      " << totalTimeOfComputeKeyPointsOctTree   / timeRecordNum << " ms" << endl;
            cout << "       | mean time of ComputeDescriptor:           " << totalTimeOfComputeDescriptor         / timeRecordNum << " ms" << endl;
            cout << "   | mean time of Track:                       " << totalTimeOfTrack                     / timeRecordNum << " ms" << endl;
            cout << "       | mean time of TrackWithIMU:                " << totalTimeOfTrackWithIMU              / timeRecordNum << " ms" << endl;
            cout << "       | mean time of TrackLocalMapWithIMU:        " << totalTimeOfTrackLocalMapWithIMU      / timeRecordNum << " ms" << endl;

            // Save camera trajectory
            SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

            // Save NavState trajectory (IMU)
            SLAM.SaveKeyFrameTrajectoryNavState("KeyFrameNavStateTrajectory.txt");

            SaveTimestamps(vImageTimestamps, "ImageTimestamps.txt");
            SaveTimestamps(vImuTimestamps, "ImuTimestamps.txt");


//            ros::shutdown();

            std::cout << std::endl << " -- Press any key at the image windown to quit" << std::endl;
            cv::waitKey(0);

            // Stop all threads
            SLAM.Shutdown();

            ros::shutdown();

            break;
        }


        ros::spinOnce();
        r.sleep();
    }


    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg, const std::vector<ORB_SLAM2::IMUData> vimuData)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonoVI(cv_ptr->image, vimuData, cv_ptr->header.stamp.toSec());
}

void SaveTimestamps(const vector<double> vTimestamps, const string &filename)
{
    ofstream f;
    f.open( filename.c_str() );
    f << fixed;

    for (size_t i=0, iend=vTimestamps.size(); i<iend; i++)
    {
        f << setprecision(6) << vTimestamps[i] << endl;
    }
    f.close();
    std::cout << std::endl << " timestamps saved in file:  " << filename << std::endl;
}
