#ifndef __SKELETON_TRACK_H
#define __SKELETON_TRACK_H

#include "orbbec_camera.h"
#include "NiTE2/NiTE.h"
#include "boost/thread.hpp"
#include "boost/bind.hpp"
#include "boost/thread/mutex.hpp"

class sk_exception:public std::exception
{
public:
    explicit sk_exception(std::string error):m_errString(error) {}
    virtual const char* what()
    {
        return m_errString.c_str();
    }
    virtual ~sk_exception()throw() {}
private:
    std::string m_errString;
};

/** \brief 人体骨骼识别与跟踪
 *
 */
class Skeleton_Track
{
public:
    Skeleton_Track(Orbbec_Camera* cam);

    ~Skeleton_Track();

    bool start_track();

    bool stop_track();

    bool is_tracking()
    {
        return m_isTracking;
    }

    vector<cv::Point3f> get_skeleton_point3f(nite::Skeleton skeleton);

    vector<cv::Point3f> get_skeleton_point3f()
    {
        return get_skeleton_point3f(m_skeleton);
    }

    vector<cv::Point2f> convert_3fto2f(vector<cv::Point3f> vec_point3f);

    void draw_skeleton(vector<cv::Point2f> vec_point2f, Mat& img);

    bool m_isTracking;

    vector<double> vec_confidence;

private:

    nite::UserTracker m_userTracker;

    boost::thread m_track_thread;

    Orbbec_Camera* m_cam;

    nite::Skeleton m_skeleton;

    void track();
};

#endif // __SKELETON_TRACK_H
