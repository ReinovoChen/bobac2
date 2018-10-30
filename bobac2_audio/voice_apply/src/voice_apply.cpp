#include "voice_apply.h"

Test::Test()
{
	//voice_sub = m_handle.subscribe("voice_file",100,&Test::callback,this);
	audio_client = m_handle.serviceClient<audio_collect::collect>("collect");   //实例化audio_client,用于后面调用collect服务
	aiui_client = m_handle.serviceClient<aiui_semantic::aiui_server>("aiui");   //实例化aiui_client,用于后面调用aiui服务
	tts_client = m_handle.serviceClient<speech_synthesis::ss_server>("tts");    //实例化tts_client,用于后面调用tts服务
	
	run();

}

Test::~Test()
{


}

void Test::run()
{
	ros::Rate loop(10);  //设置while循环里面的循环频率为10hz
	while(ros::ok()){
		audio_srv.request.collect_flag = 1;  //设置向collect服务里传入的参数,collect_flag为一表示开启语音采集
		if(audio_client.call(audio_srv)){    //调用collect服务，如果采集到一段语音的前后端点,就把这段语音储存，并返回这段语音的位置
			aiui_srv.request.audio_file = audio_srv.response.ret;   //设置向aiui服务传入的参数为collect返回的语音地址
			if(aiui_client.call(aiui_srv)){  //调用aiui服务，如果正确调用，则返回识别的语音结果和回答的语义
				if(!aiui_srv.response.nlp_str.empty()){  //如果返回的回答语义不为空,就调用tts服务将其合成为音频文件，否者就进入下一次循环
					tts_srv.request.text = aiui_srv.response.nlp_str;  //tts服务传入的参数为想要合成为语音的文字，这里取返回的回到语义
					if(tts_client.call(tts_srv)){	//调用tts服务，如果正确合成，则返回合成的音频文件的位置
						aplay_file = tts_srv.response.voice_file;
						pid_t pid;  //定义一个变量用来接受fork返回的pid
						pid = fork();  //fork一个子进程，用以播放合成的音频
						if(pid < 0){
							cout<<"fork error"<<endl;
						}	
						else if(pid == 0){  //如果返回的pid为0，就证明这是在子进程里返回的
							execlp("aplay","aplay",aplay_file.c_str(),(char*)0);  //播放合成的音频
						} 
						else if(pid > 0){  //如果返回的pid > 0则它是在父进程里返回的子进程的pid
							int exitcode = 0,ret;
							ret = wait(&exitcode);  //调用wait函数等待子进程退出并回收其资源
							if(ret == -1){
								cout<<"have no child process found"<<endl;
							}
						}
					}
				}
			}

		}
		ros::spinOnce();
		loop.sleep(); //休眠，使其循环的频率为10hz
	}
}

/*
void Test::callback(const std_msgs::String::ConstPtr& audio_file)
{
	aiui_srv.request.audio_file = audio_file->data;
	if(aiui_client.call(aiui_srv)){
		if(!aiui_srv.response.nlp_str.empty()){
			tts_srv.request.text = aiui_srv.response.nlp_str;
			if(tts_client.call(tts_srv)){
				pid_t pid;
				pid = fork();
				if(pid < 0){
					cout<<"fork error"<<endl;
				}	
				else if(pid == 0){
					execlp("aplay","aplay","/home/apng/.ros/source/AIUI/audio/tts.wav",(char*)0);
				}
				else if(pid > 0){
					int exitcode = 0,ret;
					ret = wait(&exitcode);
					if(ret == -1){
						cout<<"have no child process found"<<endl;
					}
				}
			}
		}
	}


}
*/

int main(int argc,char** argv)
{
	ros::init(argc,argv,"voice_test1"); // 初始化Ros
	Test test;
	ros::spin();
}
