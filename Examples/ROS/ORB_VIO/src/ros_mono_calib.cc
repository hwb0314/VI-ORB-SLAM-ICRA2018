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

#include <cv.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <stdio.h>
#include <stdlib.h>

//#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;
using namespace cv;

void help();

static const std::string RAW_VIDEO = "Raw Video";
static const std::string CALIBRATION = "Calibration";

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}


    ImageGrabber(int board_w_, int board_h_, int n_boards_, int board_dt_):
         board_w(board_w_), board_h(board_h_), n_boards(n_boards_), board_dt(board_dt_)
    {
        cv::namedWindow(RAW_VIDEO);
        cv::namedWindow(CALIBRATION);

        board_n = board_w * board_h;
    }

    ~ImageGrabber()
    {
        cv::destroyWindow(RAW_VIDEO);
        cv::destroyWindow(CALIBRATION);
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void setVideoWriter(string videoName, int fps, int width, int height)
    {
        CvSize size = cvSize(width, height);
        writer = new createVideoWriter( videoName, CV_FOURCC('M','J','P','G'), fps, size);
    }

    ORB_SLAM2::System* mpSLAM;

    int board_w;
    int board_h;
    int n_boards;   // Will be set by input list
    int board_dt;   // Wait 20 frames per chessboard view, 可以读入，默认为20

protected:
    int board_n;    // // 一幅图像的角点的个数
    CvVideoWriter* writer;

//    CvMat* image_points;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_calib");
    ros::start();

    if (argc != 5)
    {
        printf("ERROR: Wrong number of input parameters \n");
        help();
        return -1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // ImageGrabber igb(&SLAM);

    ImageGrabber igb(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    // SLAM.Shutdown();

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
        printf("Receiving image ... \n");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow(RAW_VIDEO, cv_ptr->image);

}


void help(){
printf("\n"
" Calling convention:\n"
" ch11_1_calib  board_w  board_h  number_of_boards  (skip_frames)\n"
" Example: rosrun ORB_SLAM2 Mono_calib 12 12 20 90"
" "
"\n"
"   WHERE:\n"
"     board_w, board_h   -- are the number of corners along the row and columns respectively\n"
"     number_of_boards   -- are the number of chessboard views to collect before calibration\n"
"     skip_frames        -- (default = 20)are the number of frames to skip before trying to collect another\n"
"                           good chessboard.  This allows you time to move the chessboard.  \n"
"                           Move it to many different locations and angles so that calibration \n"
"                           space will be well covered. \n"
"\n"
" Hit ‘p’ to pause/unpause, ESC to quit\n"
"\n");
}
