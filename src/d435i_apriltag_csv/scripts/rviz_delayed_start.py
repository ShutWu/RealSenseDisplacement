#!/usr/bin/env python3
import os
import subprocess
import rospy
from sensor_msgs.msg import Image

def main():
    rospy.init_node("rviz_delayed_start", anonymous=True)

    rviz_config = rospy.get_param("~rviz_config", "")
    wait_topic = rospy.get_param("~wait_topic", "/camera/color/image_rect_color")
    wait_timeout = float(rospy.get_param("~wait_timeout", 10.0))
    initial_delay = float(rospy.get_param("~initial_delay", 0.0))

    if initial_delay > 0:
        rospy.sleep(initial_delay)

    if wait_topic:
        try:
            rospy.loginfo("Waiting for image topic: %s", wait_topic)
            rospy.wait_for_message(wait_topic, Image, timeout=wait_timeout)
        except rospy.ROSException:
            rospy.logwarn("Timeout waiting for %s (%.1fs). Launching RViz anyway.", wait_topic, wait_timeout)

    cmd = ["rviz"]
    if rviz_config:
        cmd += ["-d", rviz_config]

    rospy.loginfo("Launching RViz: %s", " ".join(cmd))
    subprocess.Popen(cmd, env=os.environ.copy())

if __name__ == "__main__":
    main()
