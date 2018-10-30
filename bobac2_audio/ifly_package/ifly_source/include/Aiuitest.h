#ifndef AIUITEST_H_
#define AIUITEST_H_

#include "aiui/AIUI.h"
#include "FileUtil.h"
#include <unistd.h>
#include <pthread.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "jsoncpp/json/json.h"

#define TEST_ROOT_DIR "./source/AIUI/"
#define CFG_FILE_PATH "./source/AIUI/cfg/aiui.cfg"
#define LOG_DIR "./source/AIUI/log"

using namespace std;
using namespace aiui;
using namespace VA;

class VoiceListener: public IAIUIListener
{
public:
	void onEvent(const IAIUIEvent& event)const;
}; 


class VoiceSemantic
{
public:
	VoiceSemantic();
	~VoiceSemantic();
	bool voice_write(char* audio_file);
	void agent_create();
	void wakeup();
	void start();
	void stop();
	bool ret_flag;
	char *ret_iat_str;
	char *ret_nlp_str;
private:
	VoiceListener listener;
	bool w_flag;
	IAIUIAgent* magent;
	FileUtil::DataFileHelper* mFileHelper;
	
};

#endif
