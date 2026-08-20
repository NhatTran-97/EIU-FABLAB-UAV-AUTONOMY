"""Camera sources: USB, RTSP, and video files behind one interface.

Every source exposes the same contract:

    src = open_camera("2", width=640, height=480)
    ok, frame = src.read()      # (False, None) when no NEW frame is available
    src.release()

read() never returns the same frame twice. A caller driven by a ROS timer can
therefore skip work instead of republishing a stale pose.
"""
import re
import threading
import time

import cv2

RTSP_RE = re.compile(r"^(rtsp|rtmp|http|https)://", re.I)


class CameraSource:
    """Common interface. Subclasses only implement _open() and _grab()."""

    def __init__(self):
        self.cap = None
        self.resolution = None

    def _open(self):
        raise NotImplementedError

    def _grab(self):
        ok, frame = self.cap.read()
        return (ok, frame) if ok else (False, None)

    def start(self):
        self.cap = self._open()
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera source: {self}")
        ok, frame = self.cap.read()
        if not ok:
            self.release()
            raise RuntimeError(f"Cannot read the first frame from: {self}")
        self.resolution = (frame.shape[1], frame.shape[0])
        return self

    def read(self):
        ok, frame = self._grab()
        return (ok, frame, time.monotonic()) if ok else (False, None, 0.0)

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


class UsbCamera(CameraSource):
    """Local V4L2 / USB camera addressed by device index."""

    def __init__(self, index, width=None, height=None, fourcc=None, fps=None):
        super().__init__()
        self.index = int(index)
        self.width = width
        self.height = height
        self.fourcc = fourcc
        self.fps = fps

    def __str__(self):
        return f"UsbCamera(index={self.index})"

    def _open(self):
        cap = cv2.VideoCapture(self.index)
        if not cap.isOpened():
            return cap

        # Keep only the newest frame. Without this the driver queues frames and
        # latency grows without bound whenever we read slower than the camera
        # produces.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        # cap.set() fails silently on many USB cameras. The caller must compare
        # the requested size against .resolution after start().
        if self.width and self.height:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps:
            cap.set(cv2.CAP_PROP_FPS, self.fps)
        return cap


class RtspCamera(CameraSource):
    """Network stream. Must be drained continuously, so use it threaded.

    backend 'gstreamer' needs a cv2 built with GStreamer (the system python on
    this machine); the pip/conda cv2 usually only has FFMPEG.
    """

    def __init__(self, url, backend="auto", latency=100, protocol="tcp"):
        super().__init__()
        self.url = url
        self.backend = backend
        self.latency = latency
        self.protocol = protocol

    def __str__(self):
        return f"RtspCamera({self.url})"

    def _has_gstreamer(self):
        return "GStreamer:                   YES" in cv2.getBuildInformation()

    def _open(self):
        use_gst = (self.backend == "gstreamer"
                   or (self.backend == "auto" and self._has_gstreamer()))
        if not use_gst:
            return cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)

        pipeline = (f"rtspsrc location={self.url} latency={self.latency} "
            f"protocols={self.protocol} drop-on-latency=true ! "
            "decodebin ! videoconvert n-threads=4 ! video/x-raw,format=BGR ! "
            "appsink sync=false drop=true max-buffers=1")
        return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)


class FileCamera(CameraSource):
    """Video file or image sequence. Useful for replaying a recorded run."""

    def __init__(self, path, loop=True):
        super().__init__()
        self.path = path
        self.loop = loop

    def __str__(self):
        return f"FileCamera({self.path})"

    def _open(self):
        return cv2.VideoCapture(self.path)

    def _grab(self):
        ok, frame = self.cap.read()
        if not ok and self.loop:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        return (ok, frame) if ok else (False, None)


class ThreadedCamera:
    """Drain a source in a background thread, keeping only the newest frame.

    Two reasons this matters:
      - cap.read() blocks. Called straight from a ROS timer it stalls the
        executor, so every other callback in the node waits on the camera.
      - RTSP must be drained continuously or the stream falls behind and
        eventually drops.

    read() returns (False, None) when no frame has arrived since the last call,
    which lets the caller skip a cycle instead of reprocessing an old frame.
    """

    def __init__(self, source, reconnect_delay=2.0):
        self.source = source
        self.reconnect_delay = reconnect_delay
        self._lock = threading.Lock()
        self._frame = None
        self._seq = 0
        self._taken = 0
        self._running = False
        self._thread = None
        self._last_error = None
        self._frame_time = 0.0

    def __str__(self):
        return f"Threaded({self.source})"

    @property
    def resolution(self):
        return self.source.resolution

    @property
    def last_error(self):
        return self._last_error

    def start(self):
        self.source.start()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self

    def _loop(self):
        while self._running:
            try:
                ok, frame, t_cap = self.source.read()
            except Exception as e:            # keep the thread alive on driver hiccups
                self._last_error = str(e)
                ok, frame, t_cap = False, None, 0.0

            if ok:
                with self._lock:
                    self._frame = frame
                    self._frame_time = t_cap
                    self._seq += 1
                continue

            # Stream dropped: reopen after a cooldown rather than spinning.
            time.sleep(self.reconnect_delay)
            if not self._running:
                break
            try:
                self.source.release()
                self.source.start()
                self._last_error = None
            except Exception as e:
                self._last_error = str(e)

    def read(self):
        with self._lock:
            if self._seq == self._taken or self._frame is None:
                return False, None, 0.0
            self._taken = self._seq
            return True, self._frame, self._frame_time

    def release(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self.source.release()


def make_source(source, width=None, height=None, **kwargs):
    """Build the right CameraSource from a spec string, without starting it.

    "2"              -> UsbCamera(index=2)
    "rtsp://host/s"  -> RtspCamera
    "/path/clip.mp4" -> FileCamera
    """
    text = str(source).strip()
    if RTSP_RE.match(text):
        return RtspCamera(text, **{k: v for k, v in kwargs.items()
                                   if k in ("backend", "latency", "protocol")})
    if re.fullmatch(r"-?\d+", text):
        return UsbCamera(int(text), width, height,
                         **{k: v for k, v in kwargs.items()
                            if k in ("fourcc", "fps")})
    return FileCamera(text, **{k: v for k, v in kwargs.items() if k in ("loop",)})


def open_camera(source, width=None, height=None, threaded=True, **kwargs):
    """Build, wrap and start a camera source. Returns the started object."""
    src = make_source(source, width, height, **kwargs)
    # RTSP degrades badly when not drained continuously, so force threading.
    if isinstance(src, RtspCamera):
        threaded = True
    return (ThreadedCamera(src).start() if threaded else src.start())
