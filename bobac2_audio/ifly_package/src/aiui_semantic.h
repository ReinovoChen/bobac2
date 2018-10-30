#include <Aiuitest.h>
#include "ros/ros.h"
#include "voice_msgs/aiui_server.h"

class Aiui_test
{
public:
    Aiui_test();
    ~Aiui_test();

private:
    VoiceSemantic*  vs;
    ros::NodeHandle m_handle;
    ros::ServiceServer aiui_service;
    ros::Time current_time,last_time;
    bool aiui_deal(voice_msgs::aiui_server::Request &req,voice_msgs::aiui_server::Response &res);

};
