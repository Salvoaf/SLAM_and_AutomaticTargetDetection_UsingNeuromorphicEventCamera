#!/usr/bin/env python
import rospy

def initialize_node():
    rospy.init_node('bounding_box_publisher', anonymous=True)
    rospy.spin()  # Mantiene il nodo in esecuzione

if __name__ == '__main__':
    try:
        initialize_node()
    except rospy.ROSInterruptException:
        pass