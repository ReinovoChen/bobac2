#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<iostream>

using namespace std;

class TeleopJoy
{
public:
    TeleopJoy();
private:
    void callBack(const sensor_msgs::Joy::ConstPtr& joy);
    ros::NodeHandle n;
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    int i_velLinear_x, i_velLinear_y, i_velAngular; //axes number
    double f_velLinearMax, f_velAngularMAx;
};

TeleopJoy::TeleopJoy():n("~")
{
    n.param<int>("axis_linear_x",i_velLinear_x,i_velLinear_x);
    n.param<int>("axis_linear_y",i_velLinear_y,i_velLinear_y);
    n.param<int>("axis_angular",i_velAngular,i_velAngular);
    n.param<double>("linear_max", f_velLinearMax, 0.5);
    n.param<double>("angular_max", f_velAngularMAx, 3);
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
    sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::callBack, this);
}

void TeleopJoy::callBack(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist vel;
    vel.angular.z = joy->axes[i_velAngular]*f_velAngularMAx;
    vel.linear.x = joy->axes[i_velLinear_x]*f_velLinearMax;
    vel.linear.y = joy->axes[i_velLinear_y]*f_velLinearMax;
    pub.publish(vel);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleopJoy");
    TeleopJoy teleop_bobac;
    ros::spin();
}
