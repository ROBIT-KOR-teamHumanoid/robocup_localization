#!/usr/bin/python3
"""MCL / MLP 위치 오차를 MuJoCo GT 와 비교해 CSV 로 남기고, Ctrl+C 시 요약을 출력한다.

사용:
  source ~/ros2_ws/install/setup.bash
  /usr/bin/python3 tools/gt_eval.py --out /tmp/gt_eval.csv
"""
import argparse
import csv
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robocup_localization.msg import RobocupLocalization

MAP_W = 1100
MAP_H = 800
# (A, B, K) from field_coordinate.hpp
FIELDS = {
    "large": (22.0, 14.0, 1.0),
    "middle": (14.0, 9.0, 1.0),
    "small": (9.0, 6.0, 1.0),
}


def to_metres(x_px, y_px, field):
    a, b, k = field
    px_per_m_x = MAP_W / (a + 2.0 * k)
    px_per_m_y = MAP_H / (b + 2.0 * k)
    x_m = x_px / px_per_m_x - a * 0.5 - k
    y_m = b * 0.5 + k - y_px / px_per_m_y
    return x_m, y_m


class GtEval(Node):
    def __init__(self, args):
        super().__init__("gt_eval")
        self.field = FIELDS[args.field]
        self.warmup = args.warmup
        self.gt = None
        self.start = time.monotonic()
        self.errors = {"mcl": [], "mlp": []}

        self.file = open(args.out, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["t_sec", "source", "gt_x_m", "gt_y_m", "est_x_m", "est_y_m", "err_cm"])

        self.create_subscription(PoseStamped, args.gt_topic, self.on_gt, qos_profile_sensor_data)
        self.create_subscription(RobocupLocalization, args.loc_topic, self.on_mcl, 10)
        self.create_subscription(PointStamped, args.mlp_topic, self.on_mlp, 10)
        self.get_logger().info(
            f"GT={args.gt_topic}  MCL={args.loc_topic}  MLP={args.mlp_topic}  -> {args.out}")

    def on_gt(self, msg):
        self.gt = (msg.pose.position.x, msg.pose.position.y)

    def on_mcl(self, msg):
        x_m, y_m = to_metres(msg.robot_x, msg.robot_y, self.field)
        self.record("mcl", x_m, y_m)

    def on_mlp(self, msg):
        self.record("mlp", msg.point.x, msg.point.y)

    def record(self, source, x_m, y_m):
        if self.gt is None:
            return
        t = time.monotonic() - self.start
        if t < self.warmup:
            return

        err_cm = math.hypot(x_m - self.gt[0], y_m - self.gt[1]) * 100.0
        errors = self.errors[source]
        errors.append(err_cm)
        self.writer.writerow([f"{t:.3f}", source, f"{self.gt[0]:.4f}", f"{self.gt[1]:.4f}",
                              f"{x_m:.4f}", f"{y_m:.4f}", f"{err_cm:.2f}"])

        if source == "mcl" and len(errors) % 100 == 0:
            parts = []
            for name, values in self.errors.items():
                if values:
                    parts.append(f"{name} median={np.median(values):.1f}cm (n={len(values)})")
            self.get_logger().info("  ".join(parts))

    def summary(self):
        self.file.close()
        print(f"\n{'':8s} {'MCL':>10s} {'MLP':>10s}")
        rows = [
            ("n", lambda e: f"{len(e)}"),
            ("median", lambda e: f"{np.median(e):.1f}cm"),
            ("mean", lambda e: f"{e.mean():.1f}cm"),
            ("RMSE", lambda e: f"{math.sqrt((e ** 2).mean()):.1f}cm"),
            ("p90", lambda e: f"{np.percentile(e, 90):.1f}cm"),
            ("max", lambda e: f"{e.max():.1f}cm"),
            ("<20cm", lambda e: f"{(e < 20).mean() * 100:.1f}%"),
            ("<50cm", lambda e: f"{(e < 50).mean() * 100:.1f}%"),
        ]
        for label, fmt in rows:
            cells = []
            for name in ("mcl", "mlp"):
                e = np.asarray(self.errors[name])
                if len(e) == 0:
                    cells.append("-")
                else:
                    cells.append(fmt(e))
            print(f"{label:8s} {cells[0]:>10s} {cells[1]:>10s}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt-topic", default="/k1_trial/robot_pose")
    ap.add_argument("--loc-topic", default="/localization")
    ap.add_argument("--mlp-topic", default="/localization/mlp")
    ap.add_argument("--field", default="large", choices=list(FIELDS))
    ap.add_argument("--warmup", type=float, default=0.0, help="ignore the first N seconds")
    ap.add_argument("--out", default="gt_eval.csv")
    args = ap.parse_args()

    rclpy.init()
    node = GtEval(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
