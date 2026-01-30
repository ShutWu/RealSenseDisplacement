#!/usr/bin/env python3
import csv, math, os
import rospy
import tf2_ros

def main():
    rospy.init_node("tag_relative_to_csv")

    fixed_frame = rospy.get_param("~fixed_frame", "tag_fixed")
    moving_frame = rospy.get_param("~moving_frame", "tag_obj")
    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    default_csv = os.path.join(workspace_root, "output", "tag_relative.csv")
    out_csv = rospy.get_param("~output_csv", default_csv)
    rate_hz = float(rospy.get_param("~rate_hz", 30.0))

    os.makedirs(os.path.dirname(out_csv), exist_ok=True)
    f_exists = os.path.exists(out_csv) and os.path.getsize(out_csv) > 0

    f = open(out_csv, "a", newline="")
    w = csv.writer(f)
    if not f_exists:
        w.writerow(["stamp_sec", "fixed_frame", "moving_frame", "x_m", "y_m", "z_m", "range_m"])

    buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf2_ros.TransformListener(buf)

    r = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        try:
            # 最新一帧 transform（TF 会自己做链路）
            t = buf.lookup_transform(fixed_frame, moving_frame, rospy.Time(0), rospy.Duration(0.05))
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            rng = math.sqrt(x*x + y*y + z*z)
            stamp = t.header.stamp.to_sec()

            w.writerow([f"{stamp:.6f}", fixed_frame, moving_frame,
                        f"{x:.6f}", f"{y:.6f}", f"{z:.6f}", f"{rng:.6f}"])
            f.flush()
        except Exception:
            # 可能短暂丢检，忽略即可（也可以在这里写 NaN）
            pass

        r.sleep()

if __name__ == "__main__":
    main()
