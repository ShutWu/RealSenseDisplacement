#!/usr/bin/env python3
import csv, math, os
from collections import deque

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def _median(values):
    if not values:
        return None
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def _normalize_quat(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def _calibrate_static_ref(buf, camera_frame, fixed_frame, ref_frame, duration, max_samples, timeout):
    samples = []
    start = rospy.Time.now()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        if duration > 0 and (rospy.Time.now() - start).to_sec() >= duration:
            break
        if max_samples > 0 and len(samples) >= max_samples:
            break
        try:
            t = buf.lookup_transform(camera_frame, fixed_frame, rospy.Time(0), rospy.Duration(timeout))
            samples.append(t)
        except Exception:
            pass
        r.sleep()

    if not samples:
        return None

    tx = sum(s.transform.translation.x for s in samples) / len(samples)
    ty = sum(s.transform.translation.y for s in samples) / len(samples)
    tz = sum(s.transform.translation.z for s in samples) / len(samples)

    qx = sum(s.transform.rotation.x for s in samples)
    qy = sum(s.transform.rotation.y for s in samples)
    qz = sum(s.transform.rotation.z for s in samples)
    qw = sum(s.transform.rotation.w for s in samples)
    qx, qy, qz, qw = _normalize_quat(qx, qy, qz, qw)

    static_t = TransformStamped()
    static_t.header.stamp = rospy.Time.now()
    static_t.header.frame_id = camera_frame
    static_t.child_frame_id = ref_frame
    static_t.transform.translation.x = tx
    static_t.transform.translation.y = ty
    static_t.transform.translation.z = tz
    static_t.transform.rotation.x = qx
    static_t.transform.rotation.y = qy
    static_t.transform.rotation.z = qz
    static_t.transform.rotation.w = qw
    return static_t

def main():
    rospy.init_node("tag_relative_to_csv")

    fixed_frame = rospy.get_param("~fixed_frame", "tag_fixed")
    moving_frame = rospy.get_param("~moving_frame", "tag_obj")
    enable_static_ref = bool(rospy.get_param("~enable_static_ref", False))
    camera_frame = rospy.get_param("~camera_frame", "camera_color_optical_frame")
    fixed_frame_ref = rospy.get_param("~fixed_frame_ref", f"{fixed_frame}_ref")
    calib_duration = float(rospy.get_param("~calib_duration", 2.0))
    calib_samples = int(rospy.get_param("~calib_samples", 60))
    calib_timeout = float(rospy.get_param("~calib_timeout", 0.1))

    median_window = int(rospy.get_param("~median_window", 1))
    alpha = float(rospy.get_param("~alpha", 1.0))
    write_raw = bool(rospy.get_param("~write_raw", False))
    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    default_csv = os.path.join(workspace_root, "output", "tag_relative.csv")
    out_csv = rospy.get_param("~output_csv", default_csv)
    rate_hz = float(rospy.get_param("~rate_hz", 30.0))

    os.makedirs(os.path.dirname(out_csv), exist_ok=True)
    f = open(out_csv, "w", newline="")
    w = csv.writer(f)
    header = ["stamp_sec", "fixed_frame", "moving_frame", "x_m", "y_m", "z_m", "range_m"]
    if write_raw:
        header += ["raw_x_m", "raw_y_m", "raw_z_m", "raw_range_m"]
    w.writerow(header)

    buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf2_ros.TransformListener(buf)

    fixed_frame_effective = fixed_frame
    if enable_static_ref:
        rospy.loginfo("Calibrating static reference for %s -> %s", camera_frame, fixed_frame)
        static_t = _calibrate_static_ref(
            buf,
            camera_frame=camera_frame,
            fixed_frame=fixed_frame,
            ref_frame=fixed_frame_ref,
            duration=calib_duration,
            max_samples=calib_samples,
            timeout=calib_timeout,
        )
        if static_t is None:
            rospy.logwarn("No samples collected for static ref, using live %s", fixed_frame)
        else:
            tf2_ros.StaticTransformBroadcaster().sendTransform(static_t)
            fixed_frame_effective = fixed_frame_ref
            rospy.loginfo("Static reference published: %s -> %s", camera_frame, fixed_frame_ref)

    if median_window < 1:
        median_window = 1
    if alpha <= 0:
        alpha = 1.0
    if alpha > 1.0:
        alpha = 1.0

    x_hist = deque(maxlen=median_window)
    y_hist = deque(maxlen=median_window)
    z_hist = deque(maxlen=median_window)
    smooth_x = smooth_y = smooth_z = None

    r = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        try:
            # 最新一帧 transform（TF 会自己做链路）
            t = buf.lookup_transform(fixed_frame_effective, moving_frame, rospy.Time(0), rospy.Duration(0.05))
            raw_x = t.transform.translation.x
            raw_y = t.transform.translation.y
            raw_z = t.transform.translation.z

            x_hist.append(raw_x)
            y_hist.append(raw_y)
            z_hist.append(raw_z)

            med_x = _median(x_hist)
            med_y = _median(y_hist)
            med_z = _median(z_hist)

            if smooth_x is None:
                smooth_x, smooth_y, smooth_z = med_x, med_y, med_z
            else:
                smooth_x = alpha * med_x + (1.0 - alpha) * smooth_x
                smooth_y = alpha * med_y + (1.0 - alpha) * smooth_y
                smooth_z = alpha * med_z + (1.0 - alpha) * smooth_z

            rng = math.sqrt(smooth_x * smooth_x + smooth_y * smooth_y + smooth_z * smooth_z)
            stamp = t.header.stamp.to_sec()

            row = [f"{stamp:.6f}", fixed_frame_effective, moving_frame,
                   f"{smooth_x:.6f}", f"{smooth_y:.6f}", f"{smooth_z:.6f}", f"{rng:.6f}"]
            if write_raw:
                raw_rng = math.sqrt(raw_x * raw_x + raw_y * raw_y + raw_z * raw_z)
                row += [f"{raw_x:.6f}", f"{raw_y:.6f}", f"{raw_z:.6f}", f"{raw_rng:.6f}"]
            w.writerow(row)
            f.flush()
        except Exception:
            # 可能短暂丢检，忽略即可（也可以在这里写 NaN）
            pass

        r.sleep()

if __name__ == "__main__":
    main()
