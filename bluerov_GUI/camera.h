#ifndef CAMERA_H
#define CAMERA_H
#include <QPixmap>

class Camera
{
public:
    Camera() {}
    ~Camera(){}
    void initCamera();
private slots:
    void playVideo();

};
#endif // CAMERA_H
