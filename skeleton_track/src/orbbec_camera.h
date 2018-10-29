#ifndef __ORBBEC_H
#define __ORBBEC_H


//image acquire from Orbbec_Camera

#include "openni2/OpenNI.h"
#include "opencv2/opencv.hpp"
#include "iostream"
#include "exception"
#include "string"

using namespace std;
using namespace cv;
using namespace openni;

class orbbec_exception:public std::exception
{
public:
    explicit orbbec_exception(std::string error):m_errString(error) {}
    virtual const char* what()
    {
        return m_errString.c_str();
    }
    virtual ~orbbec_exception()throw() {}
private:
    std::string m_errString;
};

class Orbbec_Camera
{
public:
    Orbbec_Camera();

    virtual ~Orbbec_Camera();
public:

    Mat getDepthImage();

    Device* get_device();

private:
    Device m_device;

    VideoStream m_depth;
};


#endif //__ORBBEC_H
