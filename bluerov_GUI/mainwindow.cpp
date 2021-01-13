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

    player->setPlaylist(playlist);
    player->setVideoOutput(ui->videoWidget);

    connect(ui->videoPushButton, SIGNAL(clicked(bool)), this, SLOT(cam->pushButtonCallback()));
    connect(ui->listWidget, SIGNAL(currentRowChanged(int)), ui->stackedWidget,SLOT(setCurrentIndex(int)));
}

MainWindow::~MainWindow()
{
    delete ui;
}




