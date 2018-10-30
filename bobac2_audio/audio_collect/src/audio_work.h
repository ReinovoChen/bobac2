#include "audio_collect.h"
#include <vector>

#include "voice_msgs/collect.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;

class Work
{
public:
    Work();
    ~Work();

    AudioCollect  audio;
    vector<boost::shared_ptr<char> >  voice;
    std_msgs::String  audio_file;
    string file_name;

    ros::NodeHandle m_handle;
    ros::ServiceServer collect_service;
    bool collect_control(voice_msgs::collect::Request &req,voice_msgs::collect::Response &res);

    void run();
    //ros::Publisher m_voice_pub;
    //boost::thread pub_thread;

    //void getVoice();

};
