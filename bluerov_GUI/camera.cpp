#include"camera.h"


void Camera::init(){
    QString videoSource = "udpsrc port=5600";
    QString videoCodec = "! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264";
    QString videoDecode = "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert";
    QString videoSinkConf = "! appsink emit-signals=true sync=false max-buffers=2 drop=true";
    cmd = videoSource + " " + videoCodec + " " + videoDecode + " " + videoSinkConf;
}


