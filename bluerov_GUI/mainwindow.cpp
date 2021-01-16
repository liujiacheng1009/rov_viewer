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

//    player->setPlaylist(playlist);
//    player->setVideoOutput(ui->videoWidget);
    cam->init();
    connect(ui->videoPushButton, SIGNAL(clicked(bool)), this, SLOT(pushButtonCallback()));
    connect(ui->listWidget, SIGNAL(currentRowChanged(int)), ui->stackedWidget,SLOT(setCurrentIndex(int)));
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::pushButtonCallback(){
    cam->videoPipe = gst_parse_launch(qPrintable(cam->cmd),nullptr);
    gst_element_set_state(cam->videoPipe, GST_STATE_PLAYING);
    cam->videoSink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(cam->videoPipe),"appsink0"));
    GstAppSinkCallbacks callbacks;
    callbacks.eos = 0;
    callbacks.new_preroll = 0;
    callbacks.new_sample = videoCallback;
    gst_app_sink_set_callbacks(cam->videoSink, &callbacks,cam, nullptr);
    ui->videoLabel->setPixmap(QPixmap::fromImage(cam->frame));
}


static GstFlowReturn videoCallback(GstAppSink* appSink, gpointer data){
    GstSample *sample = gst_app_sink_pull_sample(appSink);
    GstCaps *caps = gst_sample_get_caps(sample);
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    int width= 0;
    int height = 0;
    static GstStructure *s;
    s = gst_caps_get_structure(caps, 0);
    gst_structure_get_int(s, "height", &height);
    gst_structure_get_int(s, "width", &width);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);
    Camera *cam = static_cast<Camera*>(data);
    cam->frame = QImage(map.data, width, height,QImage::Format_RGB888);
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

