#include "skeleton_track.h"

Skeleton_Track::Skeleton_Track(Orbbec_Camera* cam)
{
    m_cam = cam;
    nite::Status res = nite::NiTE::initialize();
    if (res != nite::STATUS_OK)	{
        throw sk_exception("nite Initialize failed");
    }
    m_userTracker.create(cam->get_device());
}

Skeleton_Track::~Skeleton_Track()
{
    m_userTracker.destroy();
    nite::NiTE::shutdown();
}

bool Skeleton_Track::start_track()
{
    m_track_thread = boost::thread(boost::bind(&Skeleton_Track::track, this));

}

bool Skeleton_Track::stop_track()
{
    m_track_thread.interrupt();
    m_track_thread.join();
}

void Skeleton_Track::track()
{
    nite::UserTrackerFrameRef  curr_userFrame;
    m_isTracking =  false;
    int counter = 5;
    while(true) {
        boost::this_thread::interruption_point();
        double exec_time = (double)getTickCount();
        if (m_userTracker.readFrame(&curr_userFrame) !=nite::STATUS_OK) {
            cout << "read frame failed and instead of the last one" << endl;
        };
        const nite::Array<nite::UserData>& AllUsrs = curr_userFrame.getUsers();
        for(int i=0; i < AllUsrs.getSize()&&AllUsrs.getSize()>=1; ++i) {
            const nite::UserData& cUsr = AllUsrs[i];
            if(!m_isTracking) {
                if  (cUsr.isNew()) {
                    //为新的用户建立检测姿态,当用户作出投降姿态时，启动跟踪;
                    m_userTracker.startPoseDetection( cUsr.getId(), nite::POSE_PSI );
                } else {
                    const nite::PoseData& poseData= cUsr.getPose(nite::POSE_PSI);
                    if(poseData.getType()==nite::POSE_PSI/**< 投降姿态 */ && poseData.isHeld()/**< 跟踪成功 */) {
                        //启动骨骼跟踪
                        m_userTracker.startSkeletonTracking(cUsr.getId());
                        m_isTracking = true;
                    }
                }
            } else {
                //检测到POSE_CROSSED_HANDS时，停止骨骼跟踪
                m_userTracker.startPoseDetection( cUsr.getId(), nite::POSE_CROSSED_HANDS );
                const nite::PoseData& poseData = cUsr.getPose(nite::POSE_CROSSED_HANDS);
                if(poseData.getType()==nite::POSE_CROSSED_HANDS && poseData.isHeld() ) {
                    //停止骨骼跟踪
                    m_userTracker.stopSkeletonTracking(cUsr.getId());
                    m_isTracking = false;
                }
                //获取跟踪关节
                if (cUsr.isVisible()) {
                    const nite::Skeleton& cSkeleton = cUsr.getSkeleton();
                    if( cSkeleton.getState()==nite::SKELETON_TRACKED) {
                        m_skeleton = cSkeleton;
                        counter = 5;
                        double fp = 1/(((double)getTickCount() - exec_time)*1000./getTickFrequency()/1000);
                        cout << "track fps: " << fp << endl;
                    }

                } else {
                    counter--;
                    if (!counter) {
                        m_userTracker.stopSkeletonTracking(cUsr.getId());
                        m_isTracking = false;
                        continue;
                    }
                }
                if(cUsr.isLost()) {
                    m_userTracker.stopSkeletonTracking(cUsr.getId());
                    m_isTracking = false;
                }

            }

        }//end for

    }
}


vector<cv::Point3f> Skeleton_Track::get_skeleton_point3f(nite::Skeleton skeleton)
{
    vector<cv::Point3f> vec_points;
    if(skeleton.getState() != nite::SKELETON_TRACKED ) return vec_points;
    vector<nite::SkeletonJoint> vec_joints;
    vec_points.resize(15);
    vec_joints.resize(15);

    vec_confidence.resize(15);
    vec_joints[0] = skeleton.getJoint( nite::JOINT_HEAD );
    vec_joints[1] = skeleton.getJoint( nite::JOINT_NECK );
    vec_joints[2] = skeleton.getJoint( nite::JOINT_LEFT_SHOULDER );
    vec_joints[3] = skeleton.getJoint( nite::JOINT_RIGHT_SHOULDER );
    vec_joints[4] = skeleton.getJoint( nite::JOINT_LEFT_ELBOW );
    vec_joints[5] = skeleton.getJoint( nite::JOINT_RIGHT_ELBOW );
    vec_joints[6] = skeleton.getJoint( nite::JOINT_LEFT_HAND );
    vec_joints[7] = skeleton.getJoint( nite::JOINT_RIGHT_HAND );
    vec_joints[8] = skeleton.getJoint( nite::JOINT_TORSO );
    vec_joints[9] = skeleton.getJoint( nite::JOINT_LEFT_HIP );
    vec_joints[10] = skeleton.getJoint( nite::JOINT_RIGHT_HIP );
    vec_joints[11] = skeleton.getJoint( nite::JOINT_LEFT_KNEE );
    vec_joints[12] = skeleton.getJoint( nite::JOINT_RIGHT_KNEE );
    vec_joints[13] = skeleton.getJoint( nite::JOINT_LEFT_FOOT );
    vec_joints[14] = skeleton.getJoint( nite::JOINT_RIGHT_FOOT );

    for(int i=0; i< vec_joints.size(); i++) {
        nite::Point3f p = vec_joints[i].getPosition();
        vec_points[i] = cv::Point3d(p.x, p.y, p.z);
        vec_confidence[i] = vec_joints[i].getPositionConfidence();
    }

    return vec_points;
}

vector<cv::Point2f> Skeleton_Track::convert_3fto2f(vector<cv::Point3f> vec_point3f)
{
    vector<cv::Point2f> vec_point2f;
    for( int  i = 0; i < vec_point3f.size(); ++ i ) {
        float x, y;
        m_userTracker.convertJointCoordinatesToDepth( vec_point3f[i].x, vec_point3f[i].y, vec_point3f[i].z, &x, &y);
        vec_point2f.push_back(cv::Point2f(x,y));
    }
    return vec_point2f;
}

void Skeleton_Track::draw_skeleton(vector<cv::Point2f> vec_point2f, Mat& src)
{
    if(!vec_point2f.size())
        throw sk_exception("no skeleton points");
    if(src.empty()) src = Mat(424, 512, CV_8UC3, cv::Scalar::all(0));
    if(src.channels() == 1) cvtColor(src, src, COLOR_GRAY2BGR);
    cv::line(src, vec_point2f[ 0], vec_point2f[ 1], cv::Scalar( 255, 0, 0 ), 3 );
    cv::line(src, vec_point2f[ 1], vec_point2f[ 2], cv::Scalar( 255, 0, 0 ), 3 );
    cv::line(src, vec_point2f[ 1], vec_point2f[ 3], cv::Scalar( 255, 0, 0 ), 3 );
    cv::line(src, vec_point2f[ 2], vec_point2f[ 4], cv::Scalar( 255, 0, 0 ), 3 );
    cv::line(src, vec_point2f[ 3], vec_point2f[ 5], cv::Scalar( 255, 0, 0 ), 3 );
    cv::line(src, vec_point2f[ 4], vec_point2f[ 6], cv::Scalar( 255, 0, 0 ), 3 );
    cv::line(src, vec_point2f[ 5], vec_point2f[ 7], cv::Scalar( 255, 0, 0 ), 3 );
    cv::line(src, vec_point2f[ 1], vec_point2f[ 8], cv::Scalar( 255, 0, 0 ), 3 );
    cv::line(src, vec_point2f[ 8], vec_point2f[ 9], cv::Scalar( 255, 0, 0 ), 3 );
    cv::line(src, vec_point2f[ 8], vec_point2f[10], cv::Scalar( 255, 0, 0 ), 3 );
    cv::line(src, vec_point2f[ 9], vec_point2f[11], cv::Scalar( 255, 0, 0 ), 3 );
    cv::line(src, vec_point2f[10], vec_point2f[12], cv::Scalar( 255, 0, 0 ), 3 );
    cv::line(src, vec_point2f[11], vec_point2f[13], cv::Scalar( 255, 0, 0 ), 3 );
    cv::line(src, vec_point2f[12], vec_point2f[14], cv::Scalar( 255, 0, 0 ), 3 );
    for(int i=0; i < vec_point2f.size()-1; i++) {
        if(vec_confidence[i]>0.6)
            cv::circle(src, vec_point2f[i], 3, cv::Scalar(0, 255, 0), 2);
        else cv::circle(src, vec_point2f[i], 3, cv::Scalar(0, 0, 255), 2);
    }
    vec_confidence.clear();
}

