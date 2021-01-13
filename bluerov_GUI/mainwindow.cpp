#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{

    ui->setupUi(this);
    ui->listWidget->insertItem(0,tr("参数设置"));
    ui->listWidget->insertItem(1,tr("运动控制"));
    ui->listWidget->insertItem(2,tr("显示图像"));
    ui->listWidget->insertItem(3,tr("IMU数据"));
    //cam = &Camera();
    player->setPlaylist(playlist);
    player->setVideoOutput(ui->videoWidget);
    //connect(ui->videoPushButton, SIGNAL(clicked(bool)), this, SLOT(cam->playVideo()));
    connect(ui->videoPushButton, SIGNAL(clicked(bool)), this, SLOT(playVideo1()));
    connect(ui->listWidget, SIGNAL(currentRowChanged(int)), ui->stackedWidget,SLOT(setCurrentIndex(int)));

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::playVideo()
{
    QString filename = QFileDialog::getOpenFileName(this,tr("open file"),".",tr("(*.avi *.mp4 *.flv *.mkv) | (*.*)"));
    QFileInfo fileInfo(filename);
    QUrl url = QUrl::fromLocalFile(fileInfo.absoluteFilePath());
    if (fileInfo.exists()) {
        if (fileInfo.suffix().toLower() == QLatin1String("m3u")) {
            playlist->load(url);
        } else {
            playlist->addMedia(url);
        }
    } else {
        QUrl url(filename);
        if (url.isValid()) {
            playlist->addMedia(url);
        }
    }
    player->play();
}


void MainWindow::playVideo1()
{

    //player->setMedia(QUrl("udp://192.168.2.1:5600"));
    player->setMedia(QUrl("udp://127.0.0.1:5600"));
    player->play();
}

