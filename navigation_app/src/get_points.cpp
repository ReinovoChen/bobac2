#include "map"
#include "string"
#include "list"
#include "iostream"
#include "algorithm"
#include "boost/bind.hpp"
#include "ros/ros.h"

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "navigation_app/add_pose.h"
#include "navigation_app/modify_pose.h"
#include "navigation_app/rm_pose.h"
#include "navigation_app/print_pose.h"
#include "navigation_app/pose_list.h"

struct Pose2D {
    Pose2D(double x, double y, double th):x_(x), y_(y), th_(th) {}
    double x_, y_, th_;
};

std::ostream& operator<<(std::ostream& os, const Pose2D& p)
{
    return os << '[' << p.x_ <<' ' << p.y_ << ' ' << p.th_ << ']';
}


Pose2D pose_form_ros(const geometry_msgs::Pose2D& ros_pose)
{
    return Pose2D(int(ros_pose.x*100)/100.f, int(ros_pose.y*100)/100.f, int(ros_pose.theta*100)/100.f);
}

geometry_msgs::Pose2D to_ros_pose(const Pose2D& p)
{
    geometry_msgs::Pose2D pose;
    pose.x = ((int)(p.x_*100))/100.f;
    pose.y = ((int)(p.y_*100))/100.f;
    pose.theta = (int)(p.th_*100)/100.f;
    return pose;
}


typedef std::map<std::string, std::vector<Pose2D> > NamedPoseArray;
NamedPoseArray named_pose_array;

bool operator==(const Pose2D& p1,const Pose2D& p2 )
{
    if( p1.x_ == p2.x_ && p1.y_ == p2.y_ && p1.th_ == p2.th_)
        return true;
    return false;
}

bool add_pose(navigation_app::add_pose::Request& req,
              navigation_app::add_pose::Response& res)
{

    std::string name = req.pose_list_name;
    if( name == "") {
        ROS_WARN_STREAM("name must no empty");
        res.message = "add pose failed, no name";
        res.success = false;
        return true;
    }
    ROS_DEBUG_STREAM("pose list name " << name);
    NamedPoseArray::iterator it = named_pose_array.find(name);
    if(it != named_pose_array.end()) {
        ROS_DEBUG_STREAM("find list name: " << name);
        auto it_pose = std::find(named_pose_array[name].begin(), named_pose_array[name].end(), pose_form_ros(req.pose));
        if(it_pose != named_pose_array[name].end()) {
            ROS_WARN_STREAM("the pose already in list: " << name << ", do nothing");
            res.message = std::string("the pose already in list: ") + name + std::string( ", add failed");
            res.success = false;
            return true;
        } else {
            named_pose_array[name].push_back(pose_form_ros(req.pose));
            ROS_DEBUG_STREAM("pose: " << pose_form_ros(req.pose));
        }

    } else {
        ROS_DEBUG_STREAM("cannot find list name: " << name << ", create new list");
        named_pose_array[name] = std::vector<Pose2D>();
        named_pose_array[name].push_back(pose_form_ros(req.pose));
        ROS_DEBUG_STREAM("pose: " << pose_form_ros(req.pose));
    }
    res.message = "add pose success";
    res.success = true;

    return true;
}


bool modify_pose(navigation_app::modify_pose::Request& req,
                 navigation_app::modify_pose::Response& res)
{
    std::string name = req.pose_list_name;
    if( name == "") {
        ROS_WARN_STREAM("name must no empty");
        res.message = "modify pose failed, no name";
        res.success = false;
        return true;
    }
    ROS_DEBUG_STREAM("pose list name " << name);
    NamedPoseArray::iterator it = named_pose_array.find(name);
    if(it != named_pose_array.end()) {
        ROS_DEBUG_STREAM("find list name: " << name);
        auto it_pose = std::find(named_pose_array[name].begin(), named_pose_array[name].end(), pose_form_ros(req.old_pose));
        if(it_pose != named_pose_array[name].end()) {
            *it_pose = pose_form_ros(req.new_pose);
            ROS_DEBUG_STREAM("old_pose:" << pose_form_ros(req.old_pose));
            ROS_DEBUG_STREAM("new_pose:" << pose_form_ros(req.new_pose));
            res.message = std::string("modify pose in list: ") + name + std::string(" success");
            res.success = true;
            return true;
        } else {
            ROS_WARN_STREAM("the pose in list: " << name << ", cannot find");
        }

    } else {
        ROS_DEBUG_STREAM("cannot find list: " << name);
        res.message = std::string("cannot find list: ") + name;
        res.success = false;
    }
    return true;
}


bool rm_pose(navigation_app::rm_pose::Request& req,
             navigation_app::rm_pose::Response& res)
{
    std::string name = req.pose_list_name;
    if( name == "") {
        ROS_WARN_STREAM("name must no empty");
        res.message = "failed";
        res.success = false;
        return true;
    }
    ROS_DEBUG_STREAM("pose list name " << name);
    NamedPoseArray::iterator it = named_pose_array.find(name);
    if(it != named_pose_array.end()) {
        auto it_pose = std::find(named_pose_array[name].begin(), named_pose_array[name].end(), pose_form_ros(req.pose));
        if(it_pose != named_pose_array[name].end()) {
            named_pose_array[name].erase(it_pose);
            ROS_DEBUG_STREAM("pose:" << *it_pose);
            res.message = " success";
            res.success = true;
            return true;
        } else {
            ROS_WARN_STREAM("the pose in list: " << name << ", cannot find");
        }
    } else {
        ROS_DEBUG_STREAM("cannot find list: " << name);
        res.message = "failed";
        res.success = false;
    }
    return true;
}

bool print_pose(navigation_app::print_pose::Request& req,
                navigation_app::print_pose::Response& res)
{
    std::string name = req.pose_list_name;
    if( name == "") {
        ROS_WARN_STREAM("name is empty, print all list name");
        auto it = named_pose_array.begin();
        res.list_name = "[ ";
        for(it; it != named_pose_array.end(); it++) {
            ROS_DEBUG_STREAM( it->first);
            res.list_name += std::string(it->first + std::string(" "));
        }
        res.list_name += ']';
        res.message = "success";
        res.success = true;
        return true;
    }
    ROS_DEBUG_STREAM("pose list name " << name);
    auto it = named_pose_array.find(name);
    if(it != named_pose_array.end()) {
        auto it_pose = named_pose_array[name].begin();
        for(it_pose; it_pose != named_pose_array[name].end(); it_pose++) {
            res.poses.push_back(to_ros_pose(*it_pose));
        }
        res.list_name = name;
        res.message = "success";
        res.success = true;
    } else {
        res.list_name = name;
        res.message = "failed";
        res.success = false;
    }
}


int main(int argc, char** argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::init(argc, argv, "pose_set");
    ros::NodeHandle nh;
    //ros::Subscriber sub = nh.subscribe<geometry_msgs::PointStamped>("clicked_point", 10, callback);
    ros::Publisher pub = nh.advertise<navigation_app::pose_list>("pose_set", 10);
    ros::ServiceServer add_pose_srv = nh.advertiseService("add_pose", add_pose);
    ros::ServiceServer modify_pose_srv = nh.advertiseService("modify_pose", modify_pose);
    ros::ServiceServer rm_pose_srv = nh.advertiseService("rm_pose", rm_pose);
    ros::ServiceServer print_pose_srv = nh.advertiseService("print_pose", print_pose);
    ros::Rate loop(100);
    while(ros::ok()) {
        auto it = named_pose_array.begin();
        for(it; it != named_pose_array.end(); it++) {
            navigation_app::pose_list msg;
            msg.name = it->first;
            for(auto it_pose = it->second.begin(); it_pose != it->second.end(); it_pose++) {
                msg.poses.push_back(to_ros_pose(*it_pose));
            }
            pub.publish(msg);
        }
        ros::spinOnce();
        loop.sleep();
    }

    //ros::spin();
    return 0;
}
