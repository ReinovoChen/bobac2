#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <iostream>
#include <ros/ros.h>
#include "voice_msgs/collect.h"
#include "voice_msgs/aiui_server.h"
#include "voice_msgs/ss_server.h"

using namespace std;

class Test
{
public:
    Test();
    ~Test();
private:
    string aplay_file;
    ros::NodeHandle m_handle;
    //ros::Subscriber voice_sub;
    ros::ServiceClient audio_client;
    ros::ServiceClient aiui_client;
    ros::ServiceClient tts_client;

    voice_msgs::collect  audio_srv;
    voice_msgs::aiui_server aiui_srv;
    voice_msgs::ss_server tts_srv;

    void run();
    //void callback(const std_msgs::String::ConstPtr& audio_file);

};
