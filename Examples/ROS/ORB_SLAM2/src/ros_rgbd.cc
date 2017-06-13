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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include<opencv2/core/core.hpp>

#include "../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    ros::Publisher pubPose;

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    ros::NodeHandle nh("~");

    image_transport::ImageTransport it(nh);
    image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), nh, "rgb_transport"),
                                    hintsDepth("raw", ros::TransportHints(), nh, "depth_transport");
    image_transport::SubscriberFilter rgb_sub(it, nh.resolveName("rgb_image"), 2, hintsRgb);
    image_transport::SubscriberFilter depth_sub(it, nh.resolveName("depth_image"), 2, hintsDepth);

    ROS_DEBUG("RGB transport:   %s, %d publishers on topic %s", rgb_sub.getTransport().c_str(),   rgb_sub.getNumPublishers(),   rgb_sub.getTopic().c_str());
    ROS_DEBUG("Depth transport: %s, %d publishers on topic %s", depth_sub.getTransport().c_str(), depth_sub.getNumPublishers(), depth_sub.getTopic().c_str());

    ROS_DEBUG("Vocabulary: %s", argv[1]);
    ROS_DEBUG("Settings: %s", argv[2]);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    igb.pubPose = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // pose is a homogenous rigid body transformation that describes the
    // current pose.
    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    if (pose.empty())
    {
        ROS_WARN_THROTTLE(1.0, "No pose estimate");
        return;
    }

    // About coordinate systems: The translation in Z needs to be negated,
    // otherwise the system is left-handed. Beyond that, we rotate the
    // coordinates 90 degrees about the X axis, and 90 degrees about the Z
    // axis. This results in a coordinate system where the camera's forward
    // direction is X (as seen from above), and its height is Z.

#define A(i,j) (pose.at<float>(i-1, j-1))
#define Rxx (A(1,1))
#define Rxy (A(1,2))
#define Rxz (A(1,3))
#define Ryx (A(2,1))
#define Ryy (A(2,2))
#define Ryz (A(2,3))
#define Rzx (A(3,1))
#define Rzy (A(3,2))
#define Rzz (A(3,3))

    geometry_msgs::Point positionMsg;
    cv::Mat position(pose(cv::Rect(0, 0, 3, 3)).t() * pose(cv::Rect(3, 0, 1, 3)));
#define T(i) (position.at<float>(i-1))
#define Tx  (T(1))
#define Ty  (T(2))
#define Tz  (T(3))
    positionMsg.x = -Tz;
    positionMsg.y =  Tx;
    positionMsg.z =  Ty;

    // Determine orientation quaternion from rotation submatrix. The
    // method here is taken from
    // https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Rotation_matrix_.E2.86.94_quaternion
    geometry_msgs::Quaternion orientationMsg;
    orientationMsg.w = sqrt(1.0 + Rxx + Ryy + Rzz)/2.0;
    orientationMsg.x = -(Ryx - Rxy)/4/orientationMsg.w;
    orientationMsg.y =  (Rzy - Ryz)/4/orientationMsg.w;
    orientationMsg.z =  (Rxz - Rzx)/4/orientationMsg.w;
#undef A

    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id = "slam_origin";
    poseMsg.header.stamp = ros::Time::now();
    poseMsg.pose.position = positionMsg;
    poseMsg.pose.orientation = orientationMsg;
    pubPose.publish(poseMsg);
}


