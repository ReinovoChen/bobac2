#include "nav_app_plugin.h"


std::ostream& operator<<(std::ostream& os, const Pose2D& p)
{
    return os << '[' << p.x_ <<' ' << p.y_ << ' ' << p.th_ << ']';
}

Pose2D pose_form_ros(const geometry_msgs::Pose2D& ros_pose)
{
    return Pose2D(int(100*ros_pose.x)/100.f, int(100*ros_pose.y)/100.f, int(100*ros_pose.theta)/100.f);
}

geometry_msgs::Pose2D to_ros_pose(const Pose2D& p)
{
    geometry_msgs::Pose2D pose;
    pose.x = int(100*p.x_)/100.f;
    pose.y = int(100*p.y_)/100.f;
    pose.theta = int(100*p.th_)/100.f;
    return pose;
}

void NavApp::set_pose()
{
    std::string list_name = ui->lineEdit_list_name->text().toStdString();
    //ROS_DEBUG_STREAM("list name: " << list_name);
    auto it = named_pose_array_.find(list_name);
    if (it != named_pose_array_.end()) {
        auto list = named_pose_array_[list_name];
        if (ui->tableWidget_nav_station->rowCount() != list.size()) {
            emit changeSize(list.size());
            return;
        }
        //ROS_DEBUG_STREAM("set Item");
        for(int i=0; i<list.size(); i++) {
            ui->tableWidget_nav_station->item(i, 0)->setText(QString::number(list[i].x_ ));
            ui->tableWidget_nav_station->item(i, 1)->setText(QString::number(list[i].y_ ));
            ui->tableWidget_nav_station->item(i, 2)->setText(QString::number(list[i].th_));
            ui->tableWidget_nav_station->item(i, 0)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter);
            ui->tableWidget_nav_station->item(i, 1)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter);
            ui->tableWidget_nav_station->item(i, 2)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter);
        }
    }
}

void NavApp::on_setStation()
{
    list_name_ = ui->lineEdit_list_name->text().toStdString();
    //ROS_DEBUG_STREAM("list name: " << list_name);
    auto it = named_pose_array_.find(list_name_);
    if (it != named_pose_array_.end()) {
        visualization_msgs::MarkerArray marr;
        ros::Time  t = ros::Time::now();
        int k= 0;
        for(auto it_pose = named_pose_array_[list_name_].begin();
                it_pose != named_pose_array_[list_name_].end(); it_pose++) {
            visualization_msgs::Marker mark;
            mark.type = visualization_msgs::Marker::ARROW;
            mark.frame_locked = true;
            mark.ns = "nav_station";
            mark.scale.x = 0.1;
            mark.scale.y = 0.2;
            mark.scale.z = 0.2;
            mark.color.g = 0.5;
            mark.color.a = 1.0;
            mark.color.r = 0.5;
            mark.header.frame_id = "map";
            mark.header.stamp = t;
            mark.id = k++;
            mark.action = 0;
            mark.lifetime = ros::Duration(0.1);
            geometry_msgs::Point p0, p1;
            p0.x = it_pose->x_;
            p0.y = it_pose->y_;
            p1.x = p0.x + cos(it_pose->th_);
            p1.y = p0.y + sin(it_pose->th_);
            mark.points.push_back(p0);
            mark.points.push_back(p1);
            marr.markers.push_back(mark);

        }
        //ROS_DEBUG_STREAM("marr.size(): " << marr.markers.size());
        pub_.publish(marr);
    }

}

void NavApp::on_changeSize(int size)
{
    for(int i=ui->tableWidget_nav_station->rowCount(); i>=0; i-- ) {
        ui->tableWidget_nav_station->removeRow(i);
    }
    ui->tableWidget_nav_station->setRowCount(size);
    for(int i = 0; i < size; i++) {
        ui->tableWidget_nav_station->setItem(i, 0, new QTableWidgetItem);
        ui->tableWidget_nav_station->setItem(i, 1, new QTableWidgetItem);
        ui->tableWidget_nav_station->setItem(i, 2, new QTableWidgetItem);
    }
    emit update_pose();
}

void NavApp::on_remove()
{
    ROS_DEBUG_STREAM("on remove pose");
    int pos = ui->tableWidget_nav_station->currentRow();
    ROS_DEBUG_STREAM("currentRow: " << pos);
    if(pos == -1) return;
    navigation_app::rm_pose srv;
    srv.request.pose_list_name = ui->lineEdit_list_name->text().toStdString();
    ROS_DEBUG_STREAM("remove list: "<< ui->lineEdit_list_name->text().toStdString());
    srv.request.pose.x = ui->tableWidget_nav_station->item(pos ,0)->text().toDouble();
    srv.request.pose.y = ui->tableWidget_nav_station->item(pos ,1)->text().toDouble();
    srv.request.pose.theta = ui->tableWidget_nav_station->item(pos ,2)->text().toDouble();
    if(rm_clt.call(srv)) {
        if(srv.response.success == true) {
            ROS_DEBUG_STREAM("rm pose success");
        } else {
            ROS_DEBUG_STREAM("rm pose failed");
        }
    }

}

void NavApp::destory_navapp()
{
    ROS_DEBUG_STREAM("destory nav app");
}

void NavApp::on_cellChanged(int row, int column)
{
    ROS_DEBUG_STREAM("cell changed [" << row << " " <<column << "]");
    bool ok;
    std::string list_name = ui->lineEdit_list_name->text().toStdString();
    double data = ui->tableWidget_nav_station->item(row, column)->text().toDouble(&ok);
    navigation_app::modify_pose srv;
    srv.request.pose_list_name = list_name;
    Pose2D old_pose = named_pose_array_[list_name][row];
    Pose2D new_pose = named_pose_array_[list_name][row];
    srv.request.old_pose.x = old_pose.x_;
    srv.request.old_pose.y = old_pose.y_;
    srv.request.old_pose.theta = old_pose.th_;
    srv.request.new_pose.x = old_pose.x_;
    srv.request.new_pose.y = old_pose.y_;
    srv.request.new_pose.theta = old_pose.th_;

    if(ok) {
        //ROS_DEBUG_STREAM("data: " << data<< " " << "p" << p);
        switch(column) {
        case 0:
            //p.x_  = int(data*100)/100.f;
            srv.request.new_pose.x = data;
            break;
        case 1:
            //p.y_  = int(data*100)/100.f;
            srv.request.new_pose.y = data;
            break;
        case 2:
            //p.th_  = int(data*100)/100.f;
            srv.request.new_pose.theta = data;
            break;
        }
        if(mod_clt.call(srv)) {
            if(srv.response.success) {
                ROS_DEBUG_STREAM("call modify_pose success");
            }
        }
    }
}

void NavApp::on_add()
{
    ROS_DEBUG_STREAM("on add");
    if(ui->lineEdit_list_name->text().toStdString() == "") return;
    QDialog input;
    QLabel label("X,Y,Theta");
    QLineEdit data_input;
    data_input.setText("0, 0, 0");
    QPushButton ok("OK");
    QPushButton cancel ("cancel");


    QGridLayout grid_layout;
    grid_layout.addWidget(&label, 0, 0);
    grid_layout.addWidget(&data_input, 0, 1);

    grid_layout.addWidget(&ok, 3, 1);
    grid_layout.addWidget(&cancel, 3, 0);

    input.setLayout(&grid_layout);
    connect(&ok, SIGNAL(clicked(bool)), &input, SLOT(accept()));
    connect(&cancel, SIGNAL(clicked(bool)), &input, SLOT(reject()));
    if(input.exec()) {
        auto data = data_input.text().split(',');
        bool ok_x, ok_y, ok_th;
        double x = data[0].toDouble(&ok_x);
        double y = data[1].toDouble(&ok_y);
        double th = data[2].toDouble(&ok_th);
        if(ok_x && ok_y && ok_th) {
            navigation_app::add_pose srv;
            srv.request.pose_list_name = ui->lineEdit_list_name->text().toStdString();
            srv.request.pose.x = (int)100*x/100.f;
            srv.request.pose.y = (int)100*y/100.f;
            srv.request.pose.theta = int(100*th)/100.f;
            if(add_clt.call(srv)) {
                ROS_DEBUG_STREAM("call add_pose srv success");
            } else {
                ROS_DEBUG_STREAM("call add_pose srv failed");
            }
        }
    }
}

void NavApp::on_changeListName()
{
    ROS_DEBUG_STREAM("on finshed");
    list_name_ = ui->lineEdit_list_name->text().toStdString();
}

void NavApp::manager_work()
{
    bool ok=false;
    int data = ui->lineEdit_loop_times->text().toInt(&ok);
    if(ok == false) return ;
    int loop = data;//get from ui looptimes
    auto it = named_pose_array_.find(list_name_);
    if(it == named_pose_array_.end()) {
        ROS_DEBUG_STREAM("navigation list invalid");
        return;
    }
    usleep(100);
    while(loop-- > 0) {
        ROS_DEBUG_STREAM("nav start");
        int i=0;
        for(auto it_pose = named_pose_array_[list_name_].begin();
                it_pose != named_pose_array_[list_name_].end(); it_pose++) {
            emit goalExecuting(i);
            i++;
            boost::mutex::scoped_lock manager_lock(manager_mutex_);
            current_goal_ = *it_pose;
            usleep(20);    //get current goal
            ROS_DEBUG_STREAM("[current pose, goal pose]: " << '[' << current_pose_<<" "<< current_goal_<< "]");
            exec_cond_.notify_one();
            ROS_DEBUG_STREAM("exec mutex wait...");
            manager_lock.unlock();
            boost::mutex::scoped_lock exec_lock(exec_mutex_);
            exec_cond_.wait(exec_lock);
            manager_lock.lock();
            ROS_DEBUG_STREAM("current pose: " << current_pose_);
            exec_lock.unlock();
            usleep(20);
        }
        ROS_DEBUG_STREAM("--------------------------");
    }
    ui->pushButton_start_nav->setEnabled(true);
    ui->pushButton_add_station->setEnabled(true);
    ui->pushButton_rm_station->setEnabled(true);
    emit goalExecuting(-1);
}

void NavApp::exec_work()
{
    tf::Quaternion quat;
    move_base_msgs::MoveBaseGoal goal;
    while(ros::ok()) {
        boost::mutex::scoped_lock manager_lock(manager_mutex_);
        ROS_DEBUG_STREAM("manager mutex wait...");
        exec_cond_.wait(manager_lock);
        boost::mutex::scoped_lock exec_lock(exec_mutex_);
        ROS_DEBUG_STREAM("navigation start current goal: " << current_goal_);
        quat.setRPY(0.0, 0.0, current_goal_.th_);
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.orientation.w = quat.getW();
        goal.target_pose.pose.orientation.x = quat.getX();
        goal.target_pose.pose.orientation.y = quat.getY();
        goal.target_pose.pose.orientation.z = quat.getZ();
        goal.target_pose.pose.position.x = current_goal_.x_;
        goal.target_pose.pose.position.y = current_goal_.y_;

        nav_client_->sendGoal(goal);
        isRunning_ = true;
        nav_client_->waitForResult();
        isRunning_ = false;
        if(nav_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            current_pose_ = current_goal_;
            ROS_DEBUG_STREAM("navigation finished: " << current_goal_);
        } else {
            ROS_DEBUG_STREAM("navigation failed");
        }

        ROS_DEBUG_STREAM("navigation finished: " << current_goal_);
        exec_cond_.notify_one();
        exec_lock.unlock();
        usleep(20);
        manager_lock.unlock();
    }
}

void NavApp::on_start()
{
    try {
        nav_exec_.interrupt();
    } catch (boost::thread_exception ex) {
        ROS_DEBUG_STREAM("nav_exec_ interrupt: " << ex.what());
    }
    nav_exec_.join();
    ui->pushButton_start_nav->setEnabled(false);
    ui->pushButton_rm_station->setEnabled(false);
    ui->pushButton_add_station->setEnabled(false);

    nav_exec_ = boost::thread(boost::bind(&NavApp::exec_work, this));
    nav_manager_ = boost::thread(boost::bind(&NavApp::manager_work, this));
}

void NavApp::on_stop()
{
    isRunning_ = false;
    nav_client_->cancelAllGoals();
    try {
        if(!nav_exec_.timed_join(boost::posix_time::millisec(10)));
        nav_exec_.interrupt();
    } catch (boost::thread_exception ex) {
        ROS_DEBUG_STREAM("exec thread interrupt: " << ex.what());
    }

    try {
        if(!nav_manager_.timed_join(boost::posix_time::millisec(10)));
        nav_manager_.interrupt();
    } catch (boost::thread_exception ex) {
        ROS_DEBUG_STREAM("manager thread interrupt: " << ex.what());
    }
    nav_exec_.join();
    nav_manager_.join();

    goalExecuting(-1);

    ui->pushButton_start_nav->setEnabled(true);
    ui->pushButton_add_station->setEnabled(true);
    ui->pushButton_rm_station->setEnabled(true);
    usleep(100);
    ROS_DEBUG_STREAM("on stop");
}

void NavApp::on_GoalExcuting(int row)
{
    for(int i=0; i < ui->tableWidget_nav_station->rowCount(); i++) {
        for(int j=0; j<3; j++) {
            if(i==row)
                ui->tableWidget_nav_station->item(i, j)->setBackground(QBrush(QColor(150, 150, 0)));
            else
                ui->tableWidget_nav_station->item(i, j)->setBackground(QBrush(QColor(255, 255, 255)));
        }
    }
}

void NavApp::on_statusCheck()
{
    if(!isRunning_) {
        ui->label_status->setText("not runing\n");
        ui->tableWidget_nav_station->setEditTriggers(QAbstractItemView::AllEditTriggers);
        ui->lineEdit_list_name->setEnabled(true);
        ui->lineEdit_loop_times->setEnabled(true);
        return;
    }
    ui->tableWidget_nav_station->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->lineEdit_list_name->setEnabled(false);
    ui->lineEdit_loop_times->setEnabled(false);

    actionlib::SimpleClientGoalState state = nav_client_->getState();
    if(state == actionlib::SimpleClientGoalState::PENDING)
        ui->label_status->setText("PENDING\n");
    else if(state == actionlib::SimpleClientGoalState::ACTIVE)
        ui->label_status->setText("ACTIVE\n");
    else if(state == actionlib::SimpleClientGoalState::RECALLED)
        ui->label_status->setText("RECALLED\n");
    else if(state == actionlib::SimpleClientGoalState::REJECTED)
        ui->label_status->setText("REJECTED\n");
    else if(state == actionlib::SimpleClientGoalState::PREEMPTED)
        ui->label_status->setText("PREEMPTED\n");
    else if(state == actionlib::SimpleClientGoalState::ABORTED)
        ui->label_status->setText("ABORTED\n");
    else if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        ui->label_status->setText("SUCCEEDED\n");
    else if(state == actionlib::SimpleClientGoalState::LOST)
        ui->label_status->setText("LOST\n");
    else ui->label_status->setText("UNKONWN\n");
}

void NavApp::amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
    current_pose_.x_ = int(pose->pose.pose.position.x*100)/100.f;
    current_pose_.y_ = int(pose->pose.pose.position.y*100)/100.f;
    ui->lineEdit_current_pose->setText(QString::number(current_pose_.x_)+QString(",")+QString::number(current_pose_.y_));
}

void NavApp::list_callback(const navigation_app::pose_list::ConstPtr& pose_list)
{
    std::vector<Pose2D> list;
    for(auto it = pose_list->poses.begin(); it != pose_list->poses.end(); it++) {
        list.push_back(pose_form_ros(*it));
    }
    named_pose_array_[pose_list->name] = list;
}


NavApp::NavApp(QWidget* parent):rviz::Panel(parent),ui(new Ui::Form),timer_(new QTimer)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ui->setupUi(this);
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &NavApp::amcl_callback, this);
    list_sub_ = nh_.subscribe<navigation_app::pose_list>("pose_set", 10, &NavApp::list_callback, this);

    rm_clt = nh_.serviceClient<navigation_app::rm_pose>("rm_pose");
    add_clt = nh_.serviceClient<navigation_app::add_pose>("add_pose");
    mod_clt = nh_.serviceClient<navigation_app::modify_pose>("modify_pose");

    pub_ = nh_.advertise<visualization_msgs::MarkerArray>("nav_station", 10);


    nav_client_ = NavClientPtr(new NavClient("move_base",true));
    if(!nav_client_->waitForServer(ros::Duration(5))) {
        ROS_DEBUG_STREAM("can not connect move_base server");
    }

    connect( timer_, SIGNAL(timeout()), this, SLOT(on_statusCheck()));
    connect( timer_, SIGNAL(timeout()), this, SLOT(set_pose()));
    connect( timer_, SIGNAL(timeout()), this, SLOT(on_setStation()));
    connect(this, SIGNAL(changeSize(int)), this, SLOT(on_changeSize(int)));
    connect(ui->pushButton_rm_station, SIGNAL(clicked()), this, SLOT(on_remove()));
    connect(ui->pushButton_add_station, SIGNAL(clicked()), this, SLOT(on_add()));

    connect(ui->tableWidget_nav_station, SIGNAL(cellChanged(int,int)), this, SLOT(on_cellChanged(int,int)));
    connect(ui->lineEdit_list_name, SIGNAL(editingFinished()), this,SLOT(on_changeListName()));
    connect(ui->pushButton_start_nav, SIGNAL(clicked()), this, SLOT(on_start()));
    connect(ui->pushButton_stop_nav, SIGNAL(clicked()), this, SLOT(on_stop()));
    connect(this, SIGNAL(goalExecuting(int)), this, SLOT(on_GoalExcuting(int)));
    timer_->start(100);
    current_pose_ = Pose2D(0, 0, 0);
    isRunning_ = false;
    on_stop();
    ROS_DEBUG_STREAM("Creat Nav App");
}

NavApp::~NavApp()
{
    on_stop();
    try {
        if(!nav_exec_.timed_join(boost::posix_time::millisec(1)))
            nav_exec_.interrupt();
        if(!nav_manager_.timed_join(boost::posix_time::millisec(10)));
        nav_manager_.interrupt();
    } catch (boost::thread_exception ex) {
        ROS_DEBUG_STREAM("thread interrupt: " << ex.what());
    }
    nav_exec_.join();
    nav_manager_.join();
    delete timer_;
    delete ui;
    ROS_DEBUG_STREAM("Destory Nav App");
}

#include "pluginlib/class_list_macros.h"
PLUGINLIB_EXPORT_CLASS(NavApp, rviz::Panel)
