#!/usr/bin/env python3

import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

last_transform_time = {}

def callback(msg):
    global tf_broadcaster, last_transform_time
    new_transforms = []
    for transform in msg.transforms:
        if transform.header.frame_id == "T265_odom_frame":
            transform.header.frame_id = "odom"
        if transform.child_frame_id == "T265_pose_frame":
            transform.child_frame_id = "THETA_S"

        # Check for repeated data
        transform_id = (transform.header.frame_id, transform.child_frame_id)
        if transform_id in last_transform_time:
            last_time = last_transform_time[transform_id]
            if transform.header.stamp == last_time:
                continue  # Skip redundant data
        
        last_transform_time[transform_id] = transform.header.stamp
        new_transforms.append(transform)
    
    if new_transforms:
        tf_broadcaster.sendTransform(new_transforms)

if __name__ == '__main__':
    rospy.init_node('remap_tf')
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rospy.Subscriber("/tf", TFMessage, callback)
    rospy.spin()
