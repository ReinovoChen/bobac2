#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <iostream>
#include <ros/ros.h>
#include "audio_collect/collect.h"
#include "aiui_semantic/aiui_server.h"
#include "speech_synthesis/ss_server.h"
#include "std_msgs/String.h"

using namespace std;

class Test{
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

	audio_collect::collect  audio_srv;
	aiui_semantic::aiui_server aiui_srv;
	speech_synthesis::ss_server tts_srv;
		
	void run();
	//void callback(const std_msgs::String::ConstPtr& audio_file);

};
