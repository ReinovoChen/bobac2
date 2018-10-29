#include "orbbec_camera.h"

Orbbec_Camera::Orbbec_Camera()
{
    Status rc = OpenNI::initialize();
    if(rc != STATUS_OK) {
        throw orbbec_exception("openni Initialize failed");
    }
    if(STATUS_OK!=m_device.open(ANY_DEVICE) ) {
        throw orbbec_exception("open orbbec failed");
    }
    rc = m_depth.create(m_device, SENSOR_DEPTH);
    if(STATUS_OK != rc ) {
        throw orbbec_exception("depth stream create failed");
    }
    m_depth.start();
    cout << "depth stream start" << endl;
}

Orbbec_Camera::~Orbbec_Camera()
{
    m_depth.stop();
    m_depth.destroy();
    m_device.close();
    OpenNI::shutdown();

}




Mat Orbbec_Camera::getDepthImage()
{
    Mat src;
    VideoStream* streams = &m_depth;
    VideoFrameRef frame;
    int readyStream = -1;
    for(int i=0; i<3; i++) {
        Status rc = OpenNI::waitForAnyStream(&streams, 1, &readyStream);
        if(readyStream == 0) {
            m_depth.readFrame(&frame);
            switch(frame.getVideoMode().getPixelFormat()) {
            case PIXEL_FORMAT_DEPTH_1_MM:
                src = Mat (	frame.getHeight(),
                            frame.getWidth(),
                            CV_16UC1,
                            (void*)frame.getData()  );
                break;
            default:
                break;

            }
            if (src.empty()) {
                break;
            }
        }
    }
    return src.clone();

}


Device* Orbbec_Camera::get_device()
{
    return &m_device;
}

