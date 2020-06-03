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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <numeric>
#include <functional>

#include <ros/ros.h>
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

//    if(argc != 3)
//    {
//        cerr << endl << "Usage: rosrun ORB_VIO ORB_VIO path_to_vocabulary path_to_settings" << endl;
//        ros::shutdown();
//        return 1;
//    }

    ORB_SLAM2::ConfigParam config(argv[2]);
    ORB_SLAM2::ConfigParam* pParams = new ORB_SLAM2::ConfigParam(argv[2]);

    // get bagfile path
    std::string bagfile; // = pParams->_bagfile;
    if (!pParams->_bagfile.empty())         // prior read bagfile from EuRoC.yaml
        bagfile = pParams->_bagfile;
    else                                    // else read from argv[3]
    {
//        bagfile = "/home/hri/Documents/SLAM_Dataset/EuRoC/rosbag_file/MH_01_easy.bag";
        if(argc != 4)
        {
            cerr << "\033[1m\033[31m" << "[Wrong] Should attach to a sequence name !!!" << "\033[0m" << endl;
            return 0;
        }
        bagfile = argv[3] ;
        // string sequence_name = argv[3] ;
        // bagfile = "/home/hri/Documents/SLAM_Dataset/EuRoC/rosbag_file/"+ sequence_name + ".bag";

    }


    // -----------------------------------------------------------
    /// Step 1: Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    SLAM.SetConfigParam(pParams);

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

    vector<double> vTimesOfProcessNewKeyFrame;
    vector<double> vTimesOfComputeBoW;
    vector<double> vTimesOfAssociateMapPoints;
    vector<double> vTimesOfUpdateConnections;
    vector<double> vTimesOfMapPointCulling;
    vector<double> vTimesOfCreateNewMapPoints;
    vector<double> vTimesOfSearchInNeighbors;
    vector<double> vTimesOfLocalBA;
    vector<double> vTimesOfKeyFrameCulling;

    /**
     * @brief added data sync
     */
//    double imageMsgDelaySec = config.GetImageDelayToIMU();
    double imageMsgDelaySec = pParams->GetImageDelayToIMU();
    ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
    /// Step 2: Subscribing the image and imu topics, as well as recalling the response function to record data
//    ros::NodeHandle nh;
//    ros::Subscriber imagesub = nh.subscribe("/cam0/image_raw", 3, &ORBVIO::MsgSynchronizer::imageCallback, &msgsync);
//    ros::Subscriber imusub = nh.subscribe("/imu0", 300, &ORBVIO::MsgSynchronizer::imuCallback, &msgsync);

    SensorMsgImagePtr imageMsg;
    std::vector<SensorMsgImuPtr> vimuMsg;

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
//    const bool bAccMultiply98 = config.GetAccMultiply9p8();     // default: 0
    const bool bAccMultiply98 = pParams->GetAccMultiply9p8();     // default: 0


    // read rosbag

    rosbag::Bag bag;
    bag.open(bagfile,rosbag::bagmode::Read);

    std::vector<std::string> topics;
    std::string imutopic = pParams->_imuTopic;
    std::string imagetopic = pParams->_imageTopic;
    topics.push_back(imagetopic);
    topics.push_back(imutopic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));


    ros::Rate r(1000);
    int nImages = 0;    // image number
//    while(ros::ok())
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        SensorMsgImuPtr simu = m.instantiate<SensorMsgImu>();
        if(simu!=NULL)
            msgsync.imuCallback(simu);
        SensorMsgImagePtr simage = m.instantiate<SensorMsgImage>();
        if(simage!=NULL)
                    msgsync.imageCallback(simage);

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

            ///////////////////////////////////////
            // Wait local mapping end.
            // Do not process next frame until local mapping is idle. (This can noly be used in read rosbag)
            /// Due to this waitting, the system is slow.
            if(pParams->GetWaitUntilLocalMapIdle())
            {
                bool bstop = false;
                while(!SLAM.bLocalMapAcceptKF())
                {
                    if(!ros::ok())
                    {
                        bstop=true;
                    }
                };
                if(bstop)
                    break;
            }



            // wait until LocalMapper finished CreateNewMapPoints()
//            {
//                bool bstop = false;
//                while(!SLAM.bLocalMapperCreateNewMapPointsFinished())
//                {
//                    if(!ros::ok())
//                    {
//                        bstop=true;
//                    }
//                };
//                if(bstop)
//                    break;
//            }
            ///////////////////////////////////////


            double tframe = imageMsg->header.stamp.toSec();
            vImageTimestamps.push_back( tframe );    // record stamps of new image

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            /// grab image and track
            igb.GrabImage(imageMsg, vimuData);

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000;

//            if(nImages==1)
//            {
//                std::cout << std::endl << " -- Press any key at the image windown to continue" << std::endl;
//                cv::waitKey(0);
//            }


            // Time statistic: Only record the time of successful tracking frame using vision & IMU.
            //                 Ignore the time-consuming of VI-ORB_SLAM initialization section since it uses only vision.
            if(SLAM.GetTrackWithIMUState())
            {
                vTimesTrack.push_back(ttrack);      // record track time of new image

                vTimesOfProcessingFrame.push_back(SLAM.GetTimeOfProcessingFrame());
                vTimesOfTrack.push_back(SLAM.GetTimeOfTrack());
                vTimesOfConstructFrame.push_back(SLAM.GetTimeOfConstructFrame());
                vTimesOfTrackWithIMU.push_back(SLAM.GetTimeOfTrackWithIMU());
                vTimesOfTrackLocalMapWithIMU.push_back(SLAM.GetTimeOfTrackLocalMapWithIMU());

                vTimesOfComputePyramid.push_back(SLAM.GetTimeOfComputePyramid());
                vTimesOfComputeKeyPointsOctTree.push_back(SLAM.GetTimeOfComputeKeyPointsOctTree());
                vTimesOfComputeDescriptor.push_back(SLAM.GetTImeOfComputeDescriptor());

//            }

//            if((SLAM.GetTrackWithIMUState() || SLAM.GetTrackingState()==2)  && nImages>200)
//            {
                // record the time-consuming of LocalMapping
                static double lastTimeOfProcessNewKeyFrame = 0;
                static double lastTimeOfComputeBoW = 0;
                static double lastTimeOfAssociateMapPoints = 0;
                static double lastTimeOfUpdateConnections = 0;
                static double lastTimeOfMapPointCulling = 0;
                static double lastTimeOfCreateNewMapPoints = 0;
                static double lastTimeOfSearchInNeighbors = 0;
                static double lastTimeOfLocalBA = 0;
                static double lastTimeOfKeyFrameCulling = 0;

                if(lastTimeOfProcessNewKeyFrame!=SLAM.GetTimeOfProcessNewKeyFrame()){   // time of ProcessNewKeyFrame
                    lastTimeOfProcessNewKeyFrame = SLAM.GetTimeOfProcessNewKeyFrame();
                    vTimesOfProcessNewKeyFrame.push_back(lastTimeOfProcessNewKeyFrame);
                }
                if(lastTimeOfComputeBoW!=SLAM.GetTimeOfComputeBow()) {                  // time of ComputeBow
                    lastTimeOfComputeBoW = SLAM.GetTimeOfComputeBow();
                    vTimesOfComputeBoW.push_back(lastTimeOfComputeBoW);
                }
                if(lastTimeOfAssociateMapPoints!=SLAM.GetTimeOfAssociateMapPoints()) {  // time of AssociateMapPoints
                    lastTimeOfAssociateMapPoints = SLAM.GetTimeOfAssociateMapPoints();
                    vTimesOfAssociateMapPoints.push_back(lastTimeOfAssociateMapPoints);
                }
                if(lastTimeOfUpdateConnections!=SLAM.GetTimeOfUpdateConnections()) {    // time of UpdateConnections
                    lastTimeOfUpdateConnections = SLAM.GetTimeOfUpdateConnections();
                    vTimesOfUpdateConnections.push_back(lastTimeOfUpdateConnections);
                }
                if(lastTimeOfMapPointCulling!=SLAM.GetTimeOfMapPointCulling()) {    // time of MapPointCulling
                    lastTimeOfMapPointCulling = SLAM.GetTimeOfMapPointCulling();
                    vTimesOfMapPointCulling.push_back(lastTimeOfMapPointCulling);
                }
                if(lastTimeOfCreateNewMapPoints!=SLAM.GetTimeOfCreateNewMapPoints()) {    // time of CreateNewMapPoints
                    lastTimeOfCreateNewMapPoints = SLAM.GetTimeOfCreateNewMapPoints();
                    vTimesOfCreateNewMapPoints.push_back(lastTimeOfCreateNewMapPoints);
//                    cout << "lastTimeOfCreateNewMapPoints = " << lastTimeOfCreateNewMapPoints << " ms" << endl;
                }
                if(lastTimeOfSearchInNeighbors!=SLAM.GetTimeOfSearchInNeighbors()) {    // time of SearchInNeighbors
                    lastTimeOfSearchInNeighbors = SLAM.GetTimeOfSearchInNeighbors();
                    vTimesOfSearchInNeighbors.push_back(lastTimeOfSearchInNeighbors);
                }
                if(lastTimeOfLocalBA!=SLAM.GetTimeOfLocalBA()) {    // time of LocalBA
                    lastTimeOfLocalBA = SLAM.GetTimeOfLocalBA();
                    vTimesOfLocalBA.push_back(lastTimeOfLocalBA);
                }
                if(lastTimeOfKeyFrameCulling!=SLAM.GetTimeOfKeyFrameCulling()) {    // time of LocalBA
                    lastTimeOfKeyFrameCulling = SLAM.GetTimeOfKeyFrameCulling();
                    vTimesOfKeyFrameCulling.push_back(lastTimeOfKeyFrameCulling);
                }

            }

        }

        ros::spinOnce();
        r.sleep();
        if(!ros::ok())
            break;
    }
    bag.close();

            // Stop system: if more than 10s the system do not receive any image
//            if( tTimeOfNoImage.runTime_s() > 10 && bCheckTimeOfNoImage )
    {
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


        // compute the mean time-consuming of LocalMapping
        cout << endl;

        double totalTimeOfProcessNewKeyFrame = std::accumulate(vTimesOfProcessNewKeyFrame.begin(), vTimesOfProcessNewKeyFrame.end(), 0);
        double totalTimeOfComputeBoW = std::accumulate(vTimesOfComputeBoW.begin(), vTimesOfComputeBoW.end(), 0);
        double totalTimeOfAssociateMapPoints = std::accumulate(vTimesOfAssociateMapPoints.begin(), vTimesOfAssociateMapPoints.end(), 0);
        double totalTimeOfUpdateConnections = std::accumulate(vTimesOfUpdateConnections.begin(), vTimesOfUpdateConnections.end(), 0);
        double totalTimeOfMapPointCulling = std::accumulate(vTimesOfMapPointCulling.begin(), vTimesOfMapPointCulling.end(), 0);
        double totalTimeOfCreateNewMapPoints = std::accumulate(vTimesOfCreateNewMapPoints.begin(), vTimesOfCreateNewMapPoints.end(), 0);
        double totalTimeOfSearchInNeighbors = std::accumulate(vTimesOfSearchInNeighbors.begin(), vTimesOfSearchInNeighbors.end(), 0);
        double totalTimeOfLocalBA = std::accumulate(vTimesOfLocalBA.begin(), vTimesOfLocalBA.end(), 0);
        double totalTimeOfKeyFrameCulling = std::accumulate(vTimesOfKeyFrameCulling.begin(), vTimesOfKeyFrameCulling.end(), 0);

        cout << "| the mean time-consuming of LocalMapping" << endl;
        cout << "   | mean time of ProcessNewKeyFrame:      " << totalTimeOfProcessNewKeyFrame / vTimesOfProcessNewKeyFrame.size() << " ms" << endl;
        cout << "       | mean time of ComputeBow:                  " << totalTimeOfComputeBoW         / vTimesOfComputeBoW.size() << " ms" << endl;
        cout << "       | mean time of AssociateMapPoints           " << totalTimeOfAssociateMapPoints / vTimesOfAssociateMapPoints.size() << " ms" << endl;
        cout << "       | mean time of UpdateConnections            " << totalTimeOfUpdateConnections  / vTimesOfUpdateConnections.size() << " ms" << endl;
        cout << "   | mean time of MapPointCulling:         " << totalTimeOfMapPointCulling     / vTimesOfMapPointCulling.size() << " ms" << endl;
        cout << "   | mean time of CreateNewMapPoints:      " << totalTimeOfCreateNewMapPoints  / vTimesOfCreateNewMapPoints.size() << " ms" << endl;
        cout << "   | mean time of SearchInNeighbors:       " << totalTimeOfSearchInNeighbors   / vTimesOfSearchInNeighbors.size() << " ms" << endl;
        cout << "   | mean time of LocalBA:                 " << totalTimeOfLocalBA             / vTimesOfLocalBA.size() << " ms" << endl;
        cout << "   | mean time of KeyFrameCulling:         " << totalTimeOfKeyFrameCulling     / vTimesOfKeyFrameCulling.size() << " ms" << endl;

        // Save camera trajectory
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

        // Save NavState trajectory (IMU)
        SLAM.SaveKeyFrameTrajectoryNavState("KeyFrameNavStateTrajectory.txt");

//            SaveTimestamps(vImageTimestamps, "ImageTimestamps.txt");
//            SaveTimestamps(vImuTimestamps, "ImuTimestamps.txt");
    }

//    std::cout << std::endl << " -- Press any key at the image windown to quit" << std::endl;
//    cv::waitKey(0);

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

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
