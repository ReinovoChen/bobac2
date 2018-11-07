#ifndef NAV_H
#define NAV_H

#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "ui_nav.h"
#ifndef Q_MOC_RUN
#include "rviz/panel.h"
#include "QTimer"
#include "QMessageBox"
#include "QDialog"
#endif

#include "boost/thread.hpp"
#include "boost/bind.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "navigation_app/pose_list.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "navigation_app/add_pose.h"
#include "navigation_app/modify_pose.h"
#include "navigation_app/rm_pose.h"
#include "navigation_app/print_pose.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace Ui
{
class Form;
}

struct Pose2D {
    Pose2D(double x=0, double y=0, double th=0):x_(x), y_(y), th_(th) {}
    double x_, y_, th_;
    Pose2D& operator=(const Pose2D& that)
    {
        if(this != &that) {
            this->x_ = that.x_;
            this->y_ = that.y_;
            this->th_ = that.th_;
        }
        return *this;
    }
};

typedef std::map<std::string, std::vector<Pose2D> > NamedPoseArray;

typedef boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > NavClientPtr;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> NavClient;
class NavApp:public rviz::Panel
{
    Q_OBJECT
public:
    NavApp(QWidget* parent = 0);
    virtual ~NavApp();
    Ui::Form* ui;

    QTimer* timer_;

    //GetPose* get_pose_;
    NamedPoseArray named_pose_array_;
    std::string list_name_;
    Pose2D current_goal_;
    Pose2D current_pose_;
    ros::NodeHandle nh_;
    ros::Subscriber list_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber point_sub_;

    bool isRunning_;

    ros::Publisher pub_;
    ros::Publisher goal_pub_;
    boost::thread sub_thd_;

    boost::thread nav_manager_;
    boost::thread nav_exec_;
    boost::mutex manager_mutex_;
    boost::condition_variable manager_cond_;
    boost::mutex exec_mutex_;
    boost::condition_variable exec_cond_;

    NavClientPtr nav_client_;

    ros::ServiceClient rm_clt;
    ros::ServiceClient add_clt;
    ros::ServiceClient mod_clt;

    void exec_work();
    void satus_work();
    void manager_work();
    void sub_work();

    void list_callback(const navigation_app::pose_list::ConstPtr& pose_list);
    void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
public slots:
    void set_pose();
    void destory_navapp();
    void on_changeSize(int);
    void on_remove();
    void on_cellChanged(int, int);
    void on_add();
    void on_changeListName();
    void on_goto();
    void on_setStation();
    void on_start();
    void on_stop();
    void on_GoalExcuting(int);
    void on_statusCheck();
signals:
    void changeSize(int);
    void update_pose();
    void goalExecuting(int);
};


#endif //NAV_H
