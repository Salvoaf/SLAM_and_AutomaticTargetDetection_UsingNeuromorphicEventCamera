#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class EventImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/ze_vio/event_img", Image, self.callback)
        self.min_timestamp = None

    def callback(self, data):
        # Assumi che 'header.stamp' contenga il timestamp
        current_timestamp = data.header.stamp.to_sec()  # Converti in nanosecondi per precisione

        if self.min_timestamp is None:
            self.min_timestamp = current_timestamp

        # Normalizza il timestamp
        normalized_timestamp = current_timestamp - self.min_timestamp

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rospy.loginfo(f"Received an image with normalized timestamp: {normalized_timestamp}")
            print(normalized_timestamp)
            # Puoi aggiungere qui qualsiasi elaborazione desiderata sull'immagine
            cv2.imshow("Image Window", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

def main():
    rospy.init_node('event_img_subscriber', anonymous=True)
    EventImageSubscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
