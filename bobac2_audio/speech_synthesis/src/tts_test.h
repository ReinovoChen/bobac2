#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "wav.h"

#include "ros/ros.h"
#include "voice_msgs/ss_server.h"

using namespace std;

class Tts
{
public:
    Tts();
    ~Tts();

    string tts_audio_file;
    int text_to_speech(const char* src_text,const char* des_path);
    bool s_flag;
private:
    ros::NodeHandle m_handle;
    ros::ServiceServer tts_service;
    bool tts_deal(voice_msgs::ss_server::Request &req,voice_msgs::ss_server::Response &res);

    wave_pcm_hdr default_wav_hdr;
    const char* params;
    void login();
    void logout();

};



