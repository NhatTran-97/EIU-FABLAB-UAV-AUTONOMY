import gi
import cv2
import numpy as np

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

Gst.init(None)

pipeline_str = (
    "rtspsrc location=rtsp://192.168.144.108:554/stream=1 latency=50 ! "
    "decodebin ! videoconvert ! video/x-raw,format=BGR ! "
    "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
)

pipeline = Gst.parse_launch(pipeline_str)
appsink = pipeline.get_by_name("sink")

def on_new_sample(sink):
    sample = sink.emit("pull-sample")
    if sample is None:
        return Gst.FlowReturn.ERROR

    buffer = sample.get_buffer()
    caps = sample.get_caps()
    s = caps.get_structure(0)

    width = s.get_value("width")
    height = s.get_value("height")

    ok, map_info = buffer.map(Gst.MapFlags.READ)
    if not ok:
        return Gst.FlowReturn.ERROR

    try:
        frame = np.frombuffer(map_info.data, dtype=np.uint8)
        frame = frame.reshape((height, width, 3))   # BGR

        cv2.imshow("RTSP", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            loop.quit()
    finally:
        buffer.unmap(map_info)

    return Gst.FlowReturn.OK

appsink.connect("new-sample", on_new_sample)

pipeline.set_state(Gst.State.PLAYING)

loop = GLib.MainLoop()
bus = pipeline.get_bus()
bus.add_signal_watch()

def on_message(bus, message, loop):
    t = message.type
    if t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print("ERROR:", err)
        print("DEBUG:", debug)
        loop.quit()
    elif t == Gst.MessageType.EOS:
        loop.quit()

bus.connect("message", on_message, loop)

try:
    loop.run()
except KeyboardInterrupt:
    pass
finally:
    pipeline.set_state(Gst.State.NULL)
    cv2.destroyAllWindows()


