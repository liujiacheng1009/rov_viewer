#!/usr/bin/env python3
import cv2 
import gi 
import numpy as np 

gi.require_version('Gst', '1.0')
from gi.repository import Gst 

class Video :
    def __init__(self):
        Gst.init(None)
        video_source = 'udpsrc port={}'.format(5600)
        video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        video_decode =  '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR  \
                            ! videoconvert'
        video_sink_conf = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'
        command = ' '.join([video_source, video_codec, video_decode, video_sink_conf])
        video_pipe = Gst.parse_launch(command)
        video_pipe.set_state(Gst.State.PLAYING)
        video_sink = video_pipe.get_by_name('appsink0')
        video_sink.connect('new-sample', video.callback)
        self.frame = None

    def callback(self,sink):
        sample = sink.emit('pull-sample')
        buf = sample.get_buffer()
        caps = sample.get_caps()
        self.frame = np.ndarray((
                    caps.get_structure(0).get_value('height'),
                    caps.get_structure(0).get_value('width'),3),
                    buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return Gst.FlowReturn.OK


if __name__ == "__main__":
    video = Video()
    while True:
        if(type(video.frame) != type(None)):
            cv2.imshow('frame', video.frame)
            if(cv2.waitKey(1)&0xFF==ord('q')):
                break



