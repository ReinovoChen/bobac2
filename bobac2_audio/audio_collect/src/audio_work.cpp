#include "audio_work.h"

Work::Work()
{
    collect_service = m_handle.advertiseService("collect",&Work::collect_control,this);

    ros::param::get("~audio_file",file_name);
    cout<<"----"<<file_name<<endl;
    //m_voice_pub = m_handle.advertise<std_msgs::String>("voice_file",100);
    //pub_thread = boost::thread(boost::bind(&Work::getVoice,this));

}

Work::~Work()
{

    //pub_thread.join();

}

bool Work::collect_control(voice_msgs::collect::Request &req,voice_msgs::collect::Response &res)
{
    if(req.collect_flag == 1) {
        voice = audio.get_voice();
        audio.write_wav(voice,file_name);
        res.ret = file_name;
    } else {
        cout<<"collect_flag error"<<endl;
        res.ret = "collect_flag_error";
    }

    return true;
}

/*
void Work::getVoice()
{
	string file_name = "/home/apng/.ros/source/AIUI/audio/1.wav";
	ros::Rate loop(10);
	while(ros::ok()){
		voice = audio.get_voice();
		audio.write_wav(voice,file_name);
		audio_file.data = file_name;
		m_voice_pub.publish(audio_file);

		ros::spinOnce();
		loop.sleep();
	}

}
*/

int main(int argc,char** argv)
{
    ros::init(argc,argv,"audio_collect");
    Work  voice_work;
    ros::spin();

}
