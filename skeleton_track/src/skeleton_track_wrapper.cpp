#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "skeleton_track.h"
#include "signal.h"
#include "bobac2_msgs/skeleton.h"

bool flag = true;
void sigfunc(int sig)
{
    flag = false;
}
int main(int argc, char** argv)
{
    signal(SIGINT, sigfunc);
    ros::init(argc, argv, "skeleton_track");
    ros::NodeHandle nh;
    image_transport::ImageTransport it_depth(nh), it_sk(nh);
    image_transport::Publisher depth_pub=it_depth.advertise("depth_image", 10);
    image_transport::Publisher sk_img_pub=it_sk.advertise("skeleton_image", 10);
    ros::Publisher sk_data_pub = nh.advertise<bobac2_msgs::skeleton>("skeleton", 10);
    Orbbec_Camera cam;
    Skeleton_Track track(&cam);
    namedWindow("image");
    startWindowThread();
    track.start_track();
    ros::Rate loop(20);
    while(ros::ok() && flag ) {
        Mat depth = cam.getDepthImage();
        sensor_msgs::ImagePtr img_depth;
        std_msgs::Header imgHeader;
        imgHeader.frame_id="orbbec_depth_frame";
        imgHeader.stamp=ros::Time::now();
        img_depth = cv_bridge::CvImage(imgHeader, sensor_msgs::image_encodings::MONO16, depth).toImageMsg();
        depth_pub.publish(img_depth);
        Mat img;
        depth.convertTo(img, CV_8U, 0.05);
        if (!track.is_tracking()) {
            imshow("image", img);
            waitKey(30);
            continue;
        }
        vector<cv::Point3f> points = track.get_skeleton_point3f();
        if(points.size()) {
            bobac2_msgs::skeleton sk_msg;
            sk_msg.header.frame_id = "skeleton_frame";

            sk_msg.joints.resize(15);
            for(int i=0; i<sk_msg.joints.size(); i++) {
                sk_msg.joints[i].joint_position.x = points[i].x;
                sk_msg.joints[i].joint_position.y = points[i].y;
                sk_msg.joints[i].joint_position.z = points[i].z;
                sk_msg.joints[i].confidence = track.vec_confidence[i];
            }
            sk_msg.joints[0].joint_name 	= "head";
            sk_msg.joints[1].joint_name 	= "neck";
            sk_msg.joints[2].joint_name 	= "left_shoulder";
            sk_msg.joints[3].joint_name 	= "right_shoulder";
            sk_msg.joints[4].joint_name 	= "left_elbow";
            sk_msg.joints[5].joint_name 	= "right_elbow";
            sk_msg.joints[6].joint_name 	= "left_hand";
            sk_msg.joints[7].joint_name 	= "right_hand";
            sk_msg.joints[8].joint_name 	= "torso";
            sk_msg.joints[9].joint_name 	= "left_hip";
            sk_msg.joints[10].joint_name	= "right_hip";
            sk_msg.joints[11].joint_name 	= "left_knee";
            sk_msg.joints[12].joint_name 	= "right_knee";
            sk_msg.joints[13].joint_name 	= "left_foot";
            sk_msg.joints[14].joint_name 	= "right_foot";

            sk_data_pub.publish(sk_msg);

            track.draw_skeleton(track.convert_3fto2f(points), img);
            sensor_msgs::ImagePtr img_sk;
            std_msgs::Header imgHeader;
            imgHeader.frame_id="orbbec_depth_frame";
            imgHeader.stamp=ros::Time::now();
            img_sk = cv_bridge::CvImage(imgHeader, sensor_msgs::image_encodings::MONO8, img).toImageMsg();
            depth_pub.publish(img_sk);
        }
        imshow("image", img);
        waitKey(30);
        loop.sleep();
    }
    track.stop_track();

    return 0;
}
