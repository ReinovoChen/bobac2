#include "aiui_semantic.h"

Aiui_test::Aiui_test()
{
    vs = new VoiceSemantic;
    aiui_service = m_handle.advertiseService("aiui",&Aiui_test::aiui_deal,this);

}

Aiui_test::~Aiui_test()
{
    delete vs;
}

bool Aiui_test::aiui_deal(voice_msgs::aiui_server::Request &req,voice_msgs::aiui_server::Response &res)
{
    int len = req.audio_file.length();
    char audio_path[len];
    req.audio_file.copy(audio_path,len,0);
    audio_path[len] = 0;
    cout<<"===="<<audio_path<<"==="<<endl;
    vs->voice_write(audio_path);
    last_time = ros::Time::now();
    while(1) {
        current_time = ros::Time::now();
        if(vs->ret_flag) {
            cout<<"*get aiui_semantic return empty*"<<endl;
            res.iat_str = vs->ret_iat_str;
            res.nlp_str = vs->ret_nlp_str;
            if(!res.iat_str.empty()) {
                break;
            } else {
                vs->ret_flag = false;
            }
        }
        if((current_time - last_time).toSec() > 3) {
            res.iat_str.clear();
            res.nlp_str.clear();
            cout<<"get aiui_semantic time out,have no speech voice input"<<endl;
            break;
        }
    }
    return true;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"aiui_semantic");
    Aiui_test aiui;

    ros::spin();
}
