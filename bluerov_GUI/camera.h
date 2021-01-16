#ifndef CAMERA_H
#define CAMERA_H
#include <QPixmap>
#include<gst/gst.h>
#include<gst/app/gstappsink.h>


class Camera
{
public:
    Camera() {}
    ~Camera(){}
    void init();
    QImage frame;

public:
    GstElement *videoPipe;
    GstAppSink *videoSink;
    QString cmd;

};
#endif // CAMERA_H
