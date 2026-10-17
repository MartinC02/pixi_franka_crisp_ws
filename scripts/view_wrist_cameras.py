#!/usr/bin/env python3
"""Live view of the left/right wrist (arm) and middle ZED cameras.

Opens one window with the camera RGB streams side by side, each labelled with its
topic and measured rate. A camera that is not publishing shows a "WAITING"
placeholder instead of freezing the window, so this doubles as the "is RGB
actually publishing?" check before recording.

By default it shows the POLICY VIEW: the same centre-crop-to-square and 224x224
downsample the conversion pipeline applies, so the window shows what the model
actually receives. At the ZED's 960x600 that discards 180 px from each side --
the image is cut, not squashed.

  pixi run -e humble python scripts/view_wrist_cameras.py
  ... --view raw            # full camera image
  ... --view both           # raw with the crop outlined, beside the 224x224
  ... --layout row          # cameras side by side instead of stacked
  ... --height 480          # initial pane height in px (default 600)
  ... --cameras zed_middle zed_left_arm zed_right_arm    # any subset/order

In the window:
  drag a divider     resize the cameras on either side of it
  mouse wheel        zoom that camera around the cursor
  left drag          pan a zoomed camera
  double-click       reset that camera's zoom
  L                  toggle column / row layout (row spans the monitor width)
  F / F11            toggle fullscreen
  E                  make all panes the same size again
  R                  reset zoom on all cameras
  V                  cycle view: policy -> raw -> both
  Q / Esc            quit (Ctrl-C in the terminal works too)

This is a Qt (PyQt5) window rather than cv2.imshow: OpenCV's Qt backend
hard-wires the mouse wheel to zooming the whole window, which makes per-camera
zoom impossible.
"""
import argparse
import signal
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPIC_TEMPLATE = "/zed_multi/{cam}/rgb/color/rect/image"
DEFAULT_CAMERAS = ["zed_left_arm", "zed_right_arm", "zed_middle"]

STALE_AFTER = 2.0  # seconds without a frame before a pane goes back to WAITING

POLICY_SIZE = 224  # matches resize: [224, 224] in the conversion configs

VIEWS = ("policy", "raw", "both")
ASPECT = {"policy": 1.0, "raw": 1.6, "both": 1.6 + 1.0}  # pane width / height

MAX_ZOOM = 20.0
WHEEL_STEP = 1.15  # zoom factor per wheel notch

KEY_HELP = ("wheel zoom · drag pan · dbl-click reset · L row/column · F fullscreen · "
            "E equal sizes · R reset zoom · V view · Q quit")


def policy_view(img, size=POLICY_SIZE):
    """Exactly what the policy sees: center crop to square, then resize.

    Mirrors VideoExtractor._apply_center_crop + _apply_resize in
    data_postprocessing/src/data_werkzeug/conversion/video_extractors.py for
    `center_crop: true` + `resize: [224, 224]`. Keep the two in step -- if the
    conversion config changes crop or size, this must change with it, or the
    window stops showing what the policy is actually trained on.

    At the ZED's 960x600 that crops to 600x600, discarding 180 px from each
    side -- the image is CUT, never squashed.
    """
    h, w = img.shape[:2]
    # aspect_ratio = size / size = 1.0, so the crop is the shortest edge
    if w > h:
        crop_h, crop_w = h, h
    else:
        crop_h, crop_w = w, w
    y = max(0, h // 2 - crop_h // 2)
    x = max(0, w // 2 - crop_w // 2)
    cropped = img[y:y + crop_h, x:x + crop_w]
    return cv2.resize(cropped, (size, size), interpolation=cv2.INTER_AREA), (x, y, crop_w, crop_h)


def compose(frame, view):
    """BGR image to display for `view`, built from one raw camera frame."""
    small, (cx, cy, cw, ch) = policy_view(frame)
    if view == "policy":
        return small
    if view == "both":
        # Raw with the crop region outlined, beside the 224x224 the policy
        # actually receives -- everything outside the box is discarded before
        # the model ever sees it.
        marked = frame.copy()
        cv2.rectangle(marked, (cx, cy), (cx + cw, cy + ch), (0, 200, 255), 3)
        side = cv2.resize(small, (ch, ch), interpolation=cv2.INTER_NEAREST)
        return np.hstack([marked, side])
    return frame


class CameraFeed:
    """Latest frame + rate estimate for one camera, filled from the ROS thread."""

    def __init__(self, node, cam, topic):
        self.cam = cam
        self.topic = topic
        self.bridge = CvBridge()
        self.frame = None
        self.last_stamp = 0.0
        self.hz = 0.0
        self.count = 0
        node.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # unexpected encoding -- keep the window alive
            print(f"[{self.cam}] could not convert image ({msg.encoding}): {exc}")
            return
        # One reference swap, so the GUI thread never sees a half-written frame.
        self.frame = np.ascontiguousarray(frame)
        now = time.monotonic()
        if self.last_stamp:
            dt = now - self.last_stamp
            if dt > 0:
                self.hz = 0.9 * self.hz + 0.1 * (1.0 / dt) if self.hz else 1.0 / dt
        self.last_stamp = now
        self.count += 1


class CameraView(QtWidgets.QWidget):
    """One camera pane: fit-to-pane image with its own wheel zoom and drag pan."""

    def __init__(self, feed, viewer):
        super().__init__()
        self.feed = feed
        self.viewer = viewer
        self.setMinimumSize(80, 60)
        self.reset_zoom()
        # Placement from the last paint, so mouse events map to image pixels.
        self._geom = None  # (img_w, img_h, scale, ox, oy)
        self._drag_from = None

    def reset_zoom(self):
        self.zoom = 1.0
        self.center = (0.5, 0.5)  # view centre, as a fraction of the image size
        self.update()

    # -- painting ----------------------------------------------------------
    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        W, H = self.width(), self.height()
        p.fillRect(0, 0, W, H, QtCore.Qt.black)
        feed, view = self.feed, self.viewer.view
        frame = feed.frame
        stale = time.monotonic() - feed.last_stamp > STALE_AFTER
        fs = max(11.0, min(H, W * 0.8) / 22.0)

        if frame is None or stale:
            self._geom = None
            label = "WAITING FOR FRAMES" if frame is None else "STALLED"
            self._text(p, label, 2 * fs, H / 2, fs * 1.1, QtGui.QColor(255, 0, 0))
            status = feed.topic
        else:
            img = compose(frame, view)
            self._draw_image(p, img, W, H, smooth=(view != "policy"))
            src = f"{frame.shape[1]}x{frame.shape[0]}"
            status = (f"{feed.hz:.1f} Hz  {src} -> {POLICY_SIZE}x{POLICY_SIZE}"
                      if view != "raw" else f"{feed.hz:.1f} Hz  {src}")
            if self.zoom > 1.0:
                status += f"  zoom {self.zoom:.1f}x"

        margin = max(8.0, fs * 0.6)
        self._text(p, feed.cam, margin, margin + fs, fs, QtGui.QColor(0, 255, 0))
        self._text(p, status, margin, H - margin, fs * 0.7, QtGui.QColor(230, 230, 230))
        p.end()

    def _draw_image(self, p, img, W, H, smooth):
        h, w = img.shape[:2]
        s = min(W / w, H / h) * self.zoom
        dw, dh = w * s, h * s
        # Place the image so self.center lands mid-pane, then clamp: an axis
        # smaller than the pane is centred, a larger one may not leave a gap.
        ox = W / 2 - self.center[0] * dw
        oy = H / 2 - self.center[1] * dh
        ox = (W - dw) / 2 if dw <= W else min(0.0, max(W - dw, ox))
        oy = (H - dh) / 2 if dh <= H else min(0.0, max(H - dh, oy))
        self.center = ((W / 2 - ox) / dw, (H / 2 - oy) / dh)
        self._geom = (w, h, s, ox, oy)

        # Only hand Qt the visible part of the image, so a 20x zoom does not
        # scale the whole frame up to a huge intermediate.
        sx0, sy0 = max(0.0, -ox / s), max(0.0, -oy / s)
        sx1, sy1 = min(float(w), (W - ox) / s), min(float(h), (H - oy) / s)
        if sx1 <= sx0 or sy1 <= sy0:
            return
        target = QtCore.QRectF(ox + sx0 * s, oy + sy0 * s, (sx1 - sx0) * s, (sy1 - sy0) * s)
        if smooth and s < 1.0:
            # Qt's smooth scaling is bilinear and aliases when shrinking a
            # 960x600 frame into a small pane; area averaging does not.
            crop = img[int(sy0):int(np.ceil(sy1)), int(sx0):int(np.ceil(sx1))]
            size = (max(1, round(target.width())), max(1, round(target.height())))
            img = np.ascontiguousarray(cv2.resize(crop, size, interpolation=cv2.INTER_AREA))
            qimg = QtGui.QImage(img.data, size[0], size[1], img.strides[0],
                                QtGui.QImage.Format_BGR888)
            p.drawImage(target.topLeft(), qimg)
            return
        qimg = QtGui.QImage(img.data, w, h, img.strides[0], QtGui.QImage.Format_BGR888)
        # Nearest keeps the 224 grid visible in the policy view instead of
        # smoothing it away.
        p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, smooth)
        p.drawImage(target, qimg, QtCore.QRectF(sx0, sy0, sx1 - sx0, sy1 - sy0))

    @staticmethod
    def _text(p, text, x, y, size, color):
        """Overlay text with a dark outline so it stays readable over bright scenes."""
        font = QtGui.QFont("Sans")
        font.setPixelSize(max(8, int(size)))
        font.setBold(True)
        path = QtGui.QPainterPath()
        path.addText(QtCore.QPointF(x, y), font, text)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        p.strokePath(path, QtGui.QPen(QtCore.Qt.black, max(2.0, size / 7)))
        p.fillPath(path, color)

    # -- mouse -------------------------------------------------------------
    def wheelEvent(self, event):
        if self._geom is None:
            return
        notches = event.angleDelta().y() / 120.0
        new_zoom = min(MAX_ZOOM, max(1.0, self.zoom * WHEEL_STEP ** notches))
        if new_zoom == self.zoom:
            return
        w, h, s, ox, oy = self._geom
        pos = event.position()
        # Keep the image pixel under the cursor fixed while the scale changes.
        ix, iy = (pos.x() - ox) / s, (pos.y() - oy) / s
        s_new = s * new_zoom / self.zoom
        ox_new, oy_new = pos.x() - ix * s_new, pos.y() - iy * s_new
        self.zoom = new_zoom
        self.center = ((self.width() / 2 - ox_new) / (w * s_new),
                       (self.height() / 2 - oy_new) / (h * s_new))
        self.update()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._drag_from = event.pos()
            self.setCursor(QtCore.Qt.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self._drag_from is None or self._geom is None:
            return
        w, h, s, _, _ = self._geom
        d = event.pos() - self._drag_from
        self._drag_from = event.pos()
        self.center = (self.center[0] - d.x() / (w * s), self.center[1] - d.y() / (h * s))
        self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._drag_from = None
            self.unsetCursor()

    def mouseDoubleClickEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.reset_zoom()


class Viewer(QtWidgets.QMainWindow):
    def __init__(self, feeds, view, layout, pane_height):
        super().__init__()
        self.view = view
        self.pane_height = pane_height
        self.splitter = QtWidgets.QSplitter()
        self.splitter.setChildrenCollapsible(False)
        self.splitter.setHandleWidth(6)
        self.splitter.setStyleSheet("QSplitter { background: #202020; }"
                                    "QSplitter::handle { background: #505050; }"
                                    "QSplitter::handle:hover { background: #909090; }")
        self.views = [CameraView(feed, self) for feed in feeds]
        for v in self.views:
            self.splitter.addWidget(v)
        self.setCentralWidget(self.splitter)

        for keys, slot in ((("L",), self.toggle_layout),
                           (("F", "F11"), self.toggle_fullscreen),
                           (("E",), self.equalize),
                           (("R",), self.reset_zoom),
                           (("V",), self.cycle_view),
                           (("Q", "Esc"), self.close)):
            for key in keys:
                QtWidgets.QShortcut(QtGui.QKeySequence(key), self, activated=slot)

        self.set_layout(layout, initial=True)

        # Repaint at ~30 Hz; frames arrive on the ROS thread in the background.
        self.timer = QtCore.QTimer(self, timeout=self._repaint)
        self.timer.start(33)

    def _repaint(self):
        for v in self.views:
            v.update()

    def _update_title(self):
        layout = "row" if self.splitter.orientation() == QtCore.Qt.Horizontal else "column"
        self.setWindowTitle(f"cameras [{self.view}, {layout}]  —  {KEY_HELP}")

    def set_layout(self, layout, initial=False):
        row = layout == "row"
        self.splitter.setOrientation(QtCore.Qt.Horizontal if row else QtCore.Qt.Vertical)
        self._update_title()
        if self.isFullScreen() or self.isMaximized():
            self.equalize()
            return
        n = len(self.views)
        aspect = ASPECT[self.view]
        screen = (self.screen() if not initial else QtWidgets.QApplication.primaryScreen())
        avail = screen.availableGeometry()
        if row:
            # Row: span the monitor width, pane height follows the image aspect.
            width = avail.width()
            height = min(avail.height(), int(width / n / aspect))
            self.setGeometry(avail.x(), avail.y(), width, height)
        else:
            height = min(avail.height(), self.pane_height * n)
            width = min(avail.width(), int(height / n * aspect))
            self.resize(width, height)
        self.equalize()

    def toggle_layout(self):
        row = self.splitter.orientation() == QtCore.Qt.Horizontal
        self.set_layout("column" if row else "row")

    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()
        QtCore.QTimer.singleShot(50, self.equalize)

    def equalize(self):
        self.splitter.setSizes([10_000] * len(self.views))

    def reset_zoom(self):
        for v in self.views:
            v.reset_zoom()

    def cycle_view(self):
        self.view = VIEWS[(VIEWS.index(self.view) + 1) % len(VIEWS)]
        self.reset_zoom()
        self._update_title()


def _spin(node):
    try:
        rclpy.spin(node)
    except Exception:  # shutdown from the GUI thread interrupts the wait
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cameras", nargs="+", default=DEFAULT_CAMERAS,
                        help=f"camera names to show (default: {' '.join(DEFAULT_CAMERAS)})")
    parser.add_argument("--height", type=int, default=600,
                        help="initial pane height in px for the column layout")
    parser.add_argument("--view", choices=VIEWS, default="policy",
                        help="policy: the 224x224 the model receives (default). "
                             "raw: the full camera image. "
                             "both: raw with the crop outlined, beside the 224x224.")
    parser.add_argument("--layout", choices=("column", "row"), default="column",
                        help="stack the cameras vertically (default) or side by side")
    args = parser.parse_args()

    rclpy.init()
    node = Node("wrist_camera_viewer")
    feeds = [CameraFeed(node, cam, TOPIC_TEMPLATE.format(cam=cam)) for cam in args.cameras]
    for feed in feeds:
        node.get_logger().info(f"subscribed to {feed.topic}")
    spinner = threading.Thread(target=_spin, args=(node,), daemon=True)
    spinner.start()

    app = QtWidgets.QApplication(sys.argv[:1])
    viewer = Viewer(feeds, args.view, args.layout, args.height)
    viewer.show()

    # Qt's event loop never returns to Python on its own, so Ctrl-C would be
    # swallowed; a no-op timer gives the signal handler a chance to run.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    keepalive = QtCore.QTimer(timeout=lambda: None)
    keepalive.start(200)

    try:
        app.exec_()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spinner.join(timeout=1.0)
        node.destroy_node()
    for feed in feeds:
        print(f"{feed.cam}: {feed.count} frames received")


if __name__ == "__main__":
    sys.exit(main())
