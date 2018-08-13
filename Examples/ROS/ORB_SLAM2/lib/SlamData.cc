#include "SlamData.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{

SlamData::SlamData(ORB_SLAM2::System* pSLAM, ros::NodeHandle *nodeHandler, bool bPublishROSTopic)
{
    mpSLAM = pSLAM;
    mpFrameDrawer = mpSLAM->GetpFrameDrawer();
    bEnablePublishROSTopic = bPublishROSTopic;
    // Perform tf transform and publish
    last_transform.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion q(0,0,0,1);
    last_transform.setRotation(q);

    pose_pub = (*nodeHandler).advertise<geometry_msgs::PoseStamped>("orb_slam2/pose", 1000);
    pose_inc_pub = (*nodeHandler).advertise<geometry_msgs::PoseWithCovarianceStamped>("orb_slam2/pose_cov_incr", 1000);

    all_point_cloud_pub = (*nodeHandler).advertise<sensor_msgs::PointCloud2>("orb_slam2/pointcloud_all",1);
    ref_point_cloud_pub = (*nodeHandler).advertise<sensor_msgs::PointCloud2>("orb_slam2/pointcloud_ref",1);

    image_transport::ImageTransport it_((*nodeHandler));
    current_frame_pub = it_.advertise("orb_slam2/current_frame", 1);
}

void SlamData::SaveTimePoint(TimePointIndex index)
{
    switch (index)
    {
	case TIME_BEGIN:
    	tp1 = std::chrono::steady_clock::now();
		break;
	case TIME_FINISH_CV_PROCESS:
    	tp2 = std::chrono::steady_clock::now();
		break;
	case TIME_FINISH_SLAM_PROCESS:
    	tp3 = std::chrono::steady_clock::now();
        break;
    default:
        break;
    }
}

void SlamData::CalculateAndPrintOutProcessingFrequency(void)
{
    static long spinCnt = 0;
    static double t_temp = 0;

    double time_read= std::chrono::duration_cast<std::chrono::duration<double> >(tp2 - tp1).count();
    double time_track= std::chrono::duration_cast<std::chrono::duration<double> >(tp3 - tp2).count();
    double time_total= std::chrono::duration_cast<std::chrono::duration<double> >(tp3 - tp1).count();

    ROS_DEBUG_STREAM("Image reading time = " << setw(10) << time_read  << "s" << endl);
    ROS_DEBUG_STREAM("Tracking time =      " << setw(10) << time_track << "s, frequency = " << 1/time_track << "Hz" << endl);
    ROS_DEBUG_STREAM("All cost time =      " << setw(10) << time_total << "s, frequency = " << 1/time_total << "Hz" << endl);
    t_temp = (time_total + t_temp*spinCnt)/(1+spinCnt);
    ROS_DEBUG_STREAM("Avg. time =          " << setw(10) << t_temp     << "s, frequency = " << 1/t_temp     << "Hz" << endl);
    ROS_DEBUG_STREAM("\n\n" << endl);

    spinCnt++;
}

void SlamData::PublishTFForROS(cv::Mat Tcw, cv_bridge::CvImageConstPtr cv_ptr)
{
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

    // Broadcast transform
    static tf::TransformBroadcaster br;
    new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0) * MAP_SCALE, twc.at<float>(0, 1) * MAP_SCALE, twc.at<float>(0, 2) * MAP_SCALE));
    tf::Quaternion tf_quaternion(q[0], q[1], q[2], q[3]);
    new_transform.setRotation(tf_quaternion);
    br.sendTransform(tf::StampedTransform(new_transform, ros::Time(cv_ptr->header.stamp.toSec()), "map", "camera"));
}

void SlamData::PublishPoseForROS(cv_bridge::CvImageConstPtr cv_ptr)
{
    static int frame_num = 0;
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = cv_ptr->header.stamp;
    pose.header.frame_id ="map";
    tf::poseTFToMsg(new_transform, pose.pose);
    pose_pub.publish(pose);

    geometry_msgs::PoseWithCovarianceStamped pose_inc_cov;
    pose_inc_cov.header.stamp = cv_ptr->header.stamp;
    pose_inc_cov.header.frame_id = "keyframe_" + to_string(frame_num++);
    tf::poseTFToMsg(last_transform.inverse()*new_transform, pose_inc_cov.pose.pose);
    pose_inc_cov.pose.covariance[0*7] = 0.0005;
    pose_inc_cov.pose.covariance[1*7] = 0.0005;
    pose_inc_cov.pose.covariance[2*7] = 0.0005;
    pose_inc_cov.pose.covariance[3*7] = 0.0001;
    pose_inc_cov.pose.covariance[4*7] = 0.0001;
    pose_inc_cov.pose.covariance[5*7] = 0.0001;

    pose_inc_pub.publish(pose_inc_cov);

    last_transform = new_transform;

}

void SlamData::PublishPointCloudForROS(void)
{
    sensor_msgs::PointCloud2 allMapPoints;
    sensor_msgs::PointCloud2 referenceMapPoints;
    GetCurrentROSPointCloud(allMapPoints, referenceMapPoints);
    all_point_cloud_pub.publish(allMapPoints);
    ref_point_cloud_pub.publish(referenceMapPoints);
}

void SlamData::GetCurrentROSPointCloud(sensor_msgs::PointCloud2 &all_point_cloud, sensor_msgs::PointCloud2 &ref_point_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_all( new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ref( new pcl::PointCloud<pcl::PointXYZRGB> );

    const std::vector<MapPoint*> &vpMPs = mpSLAM->GetmpMapAllMapPoints();
    const std::vector<MapPoint*> &vpRefMPs = mpSLAM->GetmpMapReferenceMapPoints();
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
    {
        return;
    }

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        pcl::PointXYZRGB p1;
        p1.x = pos.at<float>(0);
        p1.y = pos.at<float>(1);
        p1.z = pos.at<float>(2);
        p1.r = 255; p1.g = 255; p1.b = 255;
        cloud_all->points.push_back( p1 );
    }
    pcl::PCLPointCloud2 pcl_pc1;
    pcl::toPCLPointCloud2(*cloud_all, pcl_pc1);    // pcl::PointXYZRGB -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pcl_pc1, all_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    all_point_cloud.header.frame_id = "map";
    all_point_cloud.header.stamp = ros::Time::now();

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        pcl::PointXYZRGB p2;
        p2.x = pos.at<float>(0);
        p2.y = pos.at<float>(1);
        p2.z = pos.at<float>(2);
        p2.r = 255; p2.g = 0; p2.b = 0;
        cloud_ref->points.push_back( p2 );
    }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud_ref, pcl_pc2); // pcl::PointXYZRGB -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pcl_pc2, ref_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    ref_point_cloud.header.frame_id = "map";
    ref_point_cloud.header.stamp = ros::Time::now();
}

void SlamData::PublishCurrentFrameForROS(void)
{
    cv_bridge::CvImage cvi;
    cv::Mat img;
    cvi.header.frame_id = "frame";
    cvi.encoding = "bgr8";
    cvi.header.stamp = ros::Time::now();

    if (mpFrameDrawer)
    {
        img = mpFrameDrawer->DrawFrame();
        // cv::imshow("Current Frame",img);
        // cv::waitKey(1e3/FPS/2);
        cvi.image = img;
        sensor_msgs::Image im;
        cvi.toImageMsg(im);
        current_frame_pub.publish(im);
    }
}

bool SlamData::EnablePublishROSTopics(void)
{
    return bEnablePublishROSTopic;
}

} //namespace ORB_SLAM
