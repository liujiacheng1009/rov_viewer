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
private slots:
    void playVideo();
    void videoCallback();
    void pushButtonCallback();

private:
    GstElement *videoPipe;
    GstAppSink *videoSink;
    QImage frame;
    QString cmd;

};
#endif // CAMERA_H
