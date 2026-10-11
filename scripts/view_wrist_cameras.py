#!/usr/bin/env python3
"""Live stacked view of the left/right wrist (arm) ZED cameras.

Opens one OpenCV window with the wrist RGB streams in a single column, each
labelled with its topic and measured rate. A camera that is not publishing shows
a "WAITING" placeholder instead of freezing the window, so this doubles as the
"is RGB actually publishing?" check before recording.

By default it shows the POLICY VIEW: the same centre-crop-to-square and 224x224
downsample the conversion pipeline applies, so the window shows what the model
actually receives. At the ZED's 960x600 that discards 180 px from each side --
the image is cut, not squashed.

  pixi run -e humble python scripts/view_wrist_cameras.py
  ... --view raw            # full camera image, as before
  ... --view both           # raw with the crop outlined, beside the 224x224
  ... --height 480          # scale each pane to this height (default 600)
  ... --cameras zed_middle zed_left_arm zed_right_arm    # any subset/order

Press q or ESC in the window (or Ctrl-C in the terminal) to quit.
"""
import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPIC_TEMPLATE = "/zed_multi/{cam}/rgb/color/rect/image"
DEFAULT_CAMERAS = ["zed_left_arm", "zed_right_arm","zed_middle"]

STALE_AFTER = 2.0  # seconds without a frame before a pane goes back to WAITING

POLICY_SIZE = 224  # matches resize: [224, 224] in the conversion configs


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


def draw_text(img, text, org, scale, color, thickness):
    """Overlay text with a dark outline so it stays readable over bright scenes."""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0),
                thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


class CameraPane:
    """Latest frame + rate estimate for one camera."""

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
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # unexpected encoding -- keep the window alive
            print(f"[{self.cam}] could not convert image ({msg.encoding}): {exc}")
            return
        now = time.monotonic()
        if self.last_stamp:
            dt = now - self.last_stamp
            if dt > 0:
                self.hz = 0.9 * self.hz + 0.1 * (1.0 / dt) if self.hz else 1.0 / dt
        self.last_stamp = now
        self.count += 1

    def render(self, height, width, view="policy"):
        """Return a `height` x `width` BGR pane, letterboxed and annotated."""
        pane = np.zeros((height, width, 3), dtype=np.uint8)
        # Overlay text scales with the pane so it stays legible at any --height.
        fs = height / 500.0
        thick = max(1, round(height / 360))
        margin = max(10, round(height / 36))
        stale = time.monotonic() - self.last_stamp > STALE_AFTER
        if self.frame is None or stale:
            label = "WAITING FOR FRAMES" if self.frame is None else "STALLED"
            draw_text(pane, label, (margin * 2, height // 2), fs * 1.1, (0, 0, 255), thick)
            status = self.topic
        else:
            img = self.frame
            small, (cx, cy, cw, ch) = policy_view(img)
            if view == "policy":
                img = small
            elif view == "both":
                # Raw with the crop region outlined, beside the 224x224 the
                # policy actually receives -- everything outside the box is
                # discarded before the model ever sees it.
                marked = img.copy()
                cv2.rectangle(marked, (cx, cy), (cx + cw, cy + ch), (0, 200, 255), 3)
                side = cv2.resize(small, (ch, ch), interpolation=cv2.INTER_NEAREST)
                img = np.hstack([marked, side])
            scale = min(width / img.shape[1], height / img.shape[0])
            # Nearest keeps the 224 grid visible instead of smoothing it away.
            interp = cv2.INTER_NEAREST if view == "policy" else (
                cv2.INTER_LINEAR if scale > 1 else cv2.INTER_AREA)
            resized = cv2.resize(img, (int(img.shape[1] * scale), int(img.shape[0] * scale)),
                                 interpolation=interp)
            y = (height - resized.shape[0]) // 2
            x = (width - resized.shape[1]) // 2
            pane[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
            src = f"{self.frame.shape[1]}x{self.frame.shape[0]}"
            status = (f"{self.hz:.1f} Hz  {src} -> {POLICY_SIZE}x{POLICY_SIZE}"
                      if view != "raw" else f"{self.hz:.1f} Hz  {src}")
        draw_text(pane, self.cam, (margin, margin + round(fs * 25)), fs, (0, 255, 0), thick)
        draw_text(pane, status, (margin, height - margin), fs * 0.7, (230, 230, 230),
                  max(1, thick - 1))
        return pane


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cameras", nargs="+", default=DEFAULT_CAMERAS,
                        help=f"camera names to show (default: {' '.join(DEFAULT_CAMERAS)})")
    parser.add_argument("--height", type=int, default=600, help="pane height in px")
    parser.add_argument("--width", type=int, default=0,
                        help="pane width in px (default: square for policy view, else 16:10)")
    parser.add_argument("--view", choices=("policy", "raw", "both"), default="policy",
                        help="policy: the 224x224 the model receives (default). "
                             "raw: the full camera image. "
                             "both: raw with the crop outlined, beside the 224x224.")
    args = parser.parse_args()

    height = args.height
    aspect = {"policy": 1.0, "raw": 1.6, "both": 1.6 + 1.0}[args.view]
    width = args.width or int(height * aspect)

    rclpy.init()
    node = Node("wrist_camera_viewer")
    panes = [CameraPane(node, cam, TOPIC_TEMPLATE.format(cam=cam)) for cam in args.cameras]
    for pane in panes:
        node.get_logger().info(f"subscribed to {pane.topic}")

    window = f"wrist cameras [{args.view}]"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, width, height * len(panes))

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            cv2.imshow(window, np.vstack([p.render(height, width, args.view) for p in panes]))
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    for pane in panes:
        print(f"{pane.cam}: {pane.count} frames received")


if __name__ == "__main__":
    sys.exit(main())
