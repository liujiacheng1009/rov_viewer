#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QFileDialog>
#include <QMediaPlaylist>
#include "camera.h"

static GstFlowReturn videoCallback(GstAppSink* appSink, gpointer data);

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


private:
    Ui::MainWindow *ui;
    QMediaPlayer *player = new QMediaPlayer();
    QMediaPlaylist *playlist = new QMediaPlaylist();
    Camera *cam = new Camera();

private slots:
    void pushButtonCallback();



};
#endif // MAINWINDOW_H
