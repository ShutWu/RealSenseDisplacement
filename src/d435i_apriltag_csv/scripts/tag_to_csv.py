#!/usr/bin/env python3
import csv
import math
import os
import rospy
from apriltag_ros.msg import AprilTagDetectionArray

class TagCsvLogger:
    def __init__(self):
        self.output_csv = rospy.get_param("~output_csv", os.path.join(os.path.expanduser("~"), "apriltag_xyz.csv"))
        self.detections_topic = rospy.get_param("~detections_topic", "/tag_detections")
        self.allowed_ids = set(rospy.get_param("~tag_ids", [0, 1]))
        self.flush_every = int(rospy.get_param("~flush_every", 10))

        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)

        file_exists = os.path.exists(self.output_csv) and os.path.getsize(self.output_csv) > 0
        self.f = open(self.output_csv, "a", newline="")
        self.w = csv.writer(self.f)

        if not file_exists:
            # stamp_sec: ROS时间戳（秒），frame_id: 相机坐标系
            self.w.writerow(["stamp_sec", "frame_id", "tag_id", "x_m", "y_m", "z_m", "range_m"])

        self.count = 0
        self.sub = rospy.Subscriber(self.detections_topic, AprilTagDetectionArray, self.cb, queue_size=10)
        rospy.loginfo(f"[tag_to_csv] Subscribing: {self.detections_topic}")
        rospy.loginfo(f"[tag_to_csv] Writing CSV:   {self.output_csv}")
        rospy.loginfo(f"[tag_to_csv] Tag IDs:       {sorted(list(self.allowed_ids))}")

        rospy.on_shutdown(self.on_shutdown)

    def cb(self, msg: AprilTagDetectionArray):
        stamp = msg.header.stamp.to_sec()
        frame_id = msg.header.frame_id

        for det in msg.detections:
            if not det.id:
                continue
            tag_id = int(det.id[0])
            if tag_id not in self.allowed_ids:
                continue

            p = det.pose.pose.pose.position
            x, y, z = float(p.x), float(p.y), float(p.z)
            r = math.sqrt(x*x + y*y + z*z)

            self.w.writerow([f"{stamp:.6f}", frame_id, tag_id, f"{x:.6f}", f"{y:.6f}", f"{z:.6f}", f"{r:.6f}"])
            self.count += 1

        if self.count % self.flush_every == 0:
            self.f.flush()

    def on_shutdown(self):
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass

if __name__ == "__main__":
    rospy.init_node("tag_to_csv", anonymous=False)
    TagCsvLogger()
    rospy.spin()
