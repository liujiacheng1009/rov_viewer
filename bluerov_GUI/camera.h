#ifndef CAMERA_H
#define CAMERA_H
#include <QPixmap>
#include<gst/gst.h>
#include<gst/app/gstappsink.h>

static GstFlowReturn videoCallback(GstAppSink* appSink, gpointer data);

class Camera
{
public:
    Camera() {}
    ~Camera(){}
    void init();
    QImage frame;

private slots:
    void pushButtonCallback();

private:
    GstElement *videoPipe;
    GstAppSink *videoSink;
    QString cmd;

};
#endif // CAMERA_H
