#include"tts_test.h"

Tts::Tts()
{
    default_wav_hdr = {
        { 'R', 'I', 'F', 'F' },
        0,
        {'W', 'A', 'V', 'E'},
        {'f', 'm', 't', ' '},
        16,
        1,
        1,
        16000,
        32000,
        2,
        16,
        {'d', 'a', 't', 'a'},
        0

    };
    ros::param::get("~tts_audio_file",tts_audio_file);
    s_flag = false;
    login();
    tts_service = m_handle.advertiseService("tts",&Tts::tts_deal,this);
}


Tts::~Tts()
{

    logout();


}

void Tts::login()
{
    int         ret                  = MSP_SUCCESS;
    const char* login_params         = "appid = 5aab26ec, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动
    params = "voice_name = xiaoyan, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2";
    ret = MSPLogin(NULL, NULL, login_params);//第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://www.xfyun.cn注册获取

    if (MSP_SUCCESS != ret) {
        printf("TTS MSPLogin failed, error code: %d.\n", ret);
    }

}

void Tts::logout()
{
    MSPLogout();

}

int Tts::text_to_speech(const char* src_text,const char* des_path)
{
    int          ret          = -1;
    FILE*        fp           = NULL;
    const char*  sessionID    = NULL;
    unsigned int audio_len    = 0;
    wave_pcm_hdr wav_hdr      = default_wav_hdr;
    int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;
    s_flag = false;

    if (NULL == src_text || NULL == des_path) {
        printf("params is error!\n");
        return ret;
    }
    fp = fopen(des_path, "wb");
    if (NULL == fp) {
        printf("open %s error.\n", des_path);
        return ret;
    }
    /* 开始合成 */
    sessionID = QTTSSessionBegin(params, &ret);
    if (MSP_SUCCESS != ret) {
        printf("QTTSSessionBegin failed, error code: %d.\n", ret);
        fclose(fp);
        return ret;
    }
    ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
    if (MSP_SUCCESS != ret) {
        printf("QTTSTextPut failed, error code: %d.\n",ret);
        QTTSSessionEnd(sessionID, "TextPutError");
        fclose(fp);
        return ret;
    }
    printf("正在合成 ...\n");
    fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
    while (1) {
        /* 获取合成音频 */
        const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
        if (MSP_SUCCESS != ret)
            break;
        if (NULL != data) {
            fwrite(data, audio_len, 1, fp);
            wav_hdr.data_size += audio_len; //计算data_size大小
        }
        if (MSP_TTS_FLAG_DATA_END == synth_status)
            break;
        printf(">");
        usleep(150*1000); //防止频繁占用CPU
    }
    printf("\n");
    if (MSP_SUCCESS != ret) {
        printf("QTTSAudioGet failed, error code: %d.\n",ret);
        QTTSSessionEnd(sessionID, "AudioGetError");
        fclose(fp);
        return ret;
    }
    /* 修正wav文件头数据的大小 */
    wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);

    /* 将修正过的数据写回文件头部,音频文件为wav格式 */
    fseek(fp, 4, 0);
    fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
    fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
    fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
    fclose(fp);
    fp = NULL;
    /* 合成完毕 */
    ret = QTTSSessionEnd(sessionID, "Normal");
    if (MSP_SUCCESS != ret) {
        printf("QTTSSessionEnd failed, error code: %d.\n",ret);
    }

    s_flag = true;

    return ret;

}

bool Tts::tts_deal(voice_msgs::ss_server::Request &req,voice_msgs::ss_server::Response &res)
{
    text_to_speech(req.text.data(),tts_audio_file.c_str());
    while(1) {
        if(s_flag) {
            break;
        }
    }

    res.voice_file = tts_audio_file;
    return true;
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"speech_synthesis");
    Tts tts;

    ros::spin();
}
