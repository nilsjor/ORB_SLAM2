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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"
#include "../../../include/Converter.h"

#include "SlamData.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ORB_SLAM2::SlamData* pSLAMDATA)
    {
        mpSLAM = pSLAM;
        mpSLAMDATA = pSLAMDATA;
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;

    ORB_SLAM2::SlamData* mpSLAMDATA;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mono_image_grabber");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun orb_slam2 mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    ros::NodeHandle nodeHandler;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    ORB_SLAM2::SlamData SLAMDATA(&SLAM, &nodeHandler, true);
    ImageGrabber igb(&SLAM, &SLAMDATA);

    image_transport::ImageTransport imgtransport(nodeHandler);
    image_transport::Subscriber sub = imgtransport.subscribe("camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Saves 3 points of time to calculate fps: begin, finish cv process and finish SLAM process
    mpSLAMDATA->SaveTimePoint(ORB_SLAM2::SlamData::TimePointIndex::TIME_BEGIN);

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

    mpSLAMDATA->SaveTimePoint(ORB_SLAM2::SlamData::TimePointIndex::TIME_FINISH_CV_PROCESS);

    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

    mpSLAMDATA->SaveTimePoint(ORB_SLAM2::SlamData::TimePointIndex::TIME_FINISH_SLAM_PROCESS);

    mpSLAMDATA->CalculateAndPrintOutProcessingFrequency();

    if (Tcw.empty()) {
      return;
    }

    if (mpSLAMDATA->EnablePublishROSTopics())
    {
        mpSLAMDATA->PublishTFForROS(Tcw, cv_ptr);
        mpSLAMDATA->PublishPoseForROS(cv_ptr);
        mpSLAMDATA->PublishPointCloudForROS();
        mpSLAMDATA->PublishCurrentFrameForROS();
    }
}
