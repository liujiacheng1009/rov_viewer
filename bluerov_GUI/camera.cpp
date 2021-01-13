#include"camera.h"


void Camera::init(){
    QString videoSource = "udpsrc port=5600";
    QString videoCodec = "! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264";
    QString videoDecode = "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert";
    QString videoSinkConf = "! appsink emit-signals=true sync=false max-buffers=2 drop=true";
    cmd = videoSource + " " + videoCodec + " " + videoDecode + " " + videoSinkConf;
}

void Camera::pushButtonCallback(){
    videoPipe = gst_parse_launch(qPrintable(cmd),nullptr);
    gst_element_set_state(videoPipe, GST_STATE_PLAYING);
    videoSink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(videoPipe),"appsink0"));
    GstAppSinkCallbacks callbacks;
    callbacks.eos = 0;
    callbacks.new_preroll = 0;
    callbacks.new_sample = videoCallback;
    gst_app_sink_set_callbacks(videoSink, &callbacks,this, nullptr);
}

GstFlowReturn videoCallback(GstAppSink* appSink, gpointer data){
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
