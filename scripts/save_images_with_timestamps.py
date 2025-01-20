#!/usr/bin/env python3
import rospy
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageSaverWithTimestamp:
    def __init__(self, output_dir, queue_size):
        self.output_dir = output_dir
        self.rgb_dir = os.path.join(self.output_dir, 'rgb')
        if not os.path.exists(self.rgb_dir):
            os.makedirs(self.rgb_dir)
        self.rgb_txt_path = os.path.join(self.output_dir, 'rgb.txt')
        self.bridge = CvBridge()
        self.frame_counter = 0

        rospy.Subscriber("/camera/image_raw", Image, self.image_callback, queue_size=queue_size)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            timestamp = data.header.stamp.to_sec()
            filename = f"frame{self.frame_counter:04d}.jpg"
            file_path = os.path.join(self.rgb_dir, filename)

            cv2.imwrite(file_path, cv_image)

            with open(self.rgb_txt_path, 'a') as f:
                f.write(f"{timestamp} rgb/{filename}\n")

            self.frame_counter += 1
            rospy.loginfo(f"Saved image {file_path}")

        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('image_saver_with_timestamp', anonymous=True)
    output_dir = rospy.get_param('~output_image_path', '/tmp')
    queue_size = rospy.get_param('~queue_size', 100000)
    ImageSaverWithTimestamp(output_dir, queue_size)
    rospy.spin()
