import rospy
from sensor_msgs.msg import Image
from hello_world.msg import BoundingBoxWithTime  # Importa il messaggio custom corretto
from cv_bridge import CvBridge
import cv2
from collections import deque

frame_queue = deque()
bbox_list = []
sync_image_pub = None
publish_rate = 10  # Frequenza di pubblicazione in Hz
first_image_timestamp = None
Tw = 10
def image_callback(img_msg):
    global frame_queue, first_image_timestamp
    rospy.loginfo("Ricevuto nuovo frame.")
    frame = CvBridge().imgmsg_to_cv2(img_msg, "mono8")
    frame_timestamp = img_msg.header.stamp.to_sec()

    if first_image_timestamp is None:
        first_image_timestamp = frame_timestamp
        rospy.loginfo(f"Timestamp del primo frame: {first_image_timestamp}")

    normalized_timestamp = frame_timestamp - first_image_timestamp
    frame_queue.append((normalized_timestamp, frame))
    rospy.loginfo(f"Frame aggiunto alla coda con timestamp normalizzato: {normalized_timestamp}")

    # Rimuovere vecchi frame oltre i 50 secondi
    while frame_queue and (normalized_timestamp - frame_queue[0][0] > 50):
        rospy.loginfo(f"Rimuovo vecchio frame con timestamp: {frame_queue[0][0]}")
        frame_queue.popleft()

def bbox_callback(bbox_msg):
    global bbox_list
    rospy.loginfo("Ricevuto nuovo bounding box.")
    bbox_list.append(bbox_msg)
    rospy.loginfo(f"Bounding box aggiunto alla lista con timestamp: {bbox_msg.timestamp}")

    # Rimuovere bounding box obsolete
    min_frame_timestamp = frame_queue[0][0] if frame_queue else 0
    bbox_list = [bbox for bbox in bbox_list if bbox.timestamp >= min_frame_timestamp]
    rospy.loginfo("Bounding box obsoleti rimossi dalla lista.")

def synchronize_and_publish():
    global frame_queue, bbox_list, sync_image_pub

    if len(frame_queue) == 0:
        return

    current_time = frame_queue[-1][0]
    rospy.loginfo(f"Sincronizzazione e pubblicazione per timestamp corrente: {current_time}")
    print(current_time, frame_queue[0][0] )
    # Sincronizzare e pubblicare frame
    while frame_queue and (current_time - frame_queue[0][0] >= Tw):
        frame_timestamp, frame = frame_queue.popleft()
        closest_bbox = None
        min_diff = float('inf')

        for bbox in bbox_list:
            diff = abs(bbox.timestamp - frame_timestamp)
            if diff < min_diff:
                min_diff = diff
                closest_bbox = bbox

        if closest_bbox and closest_bbox.class_id == 1:
            # Convertire il frame da scala di grigi a BGR
            frame_color = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            # Disegnare la bounding box blu sul frame
            x, y, w, h = closest_bbox.x_offset, closest_bbox.y_offset, closest_bbox.width, closest_bbox.height
            cv2.rectangle(frame_color, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Bounding box blu
            rospy.loginfo(f"Bounding box disegnato sul frame con timestamp: {frame_timestamp}")
        else:
            frame_color = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # Convertire il frame in un messaggio ROS e pubblicarlo
        sync_image_msg = CvBridge().cv2_to_imgmsg(frame_color, "bgr8")
        sync_image_pub.publish(sync_image_msg)
        rospy.loginfo("Frame sincronizzato pubblicato.")

def main():
    global sync_image_pub

    rospy.init_node('image_bbox_synchronizer')

    rospy.Subscriber('/ze_vio/event_img', Image, image_callback)
    rospy.Subscriber('/bounding_box_with_time', BoundingBoxWithTime, bbox_callback)

    sync_image_pub = rospy.Publisher('/synchronized_images', Image, queue_size=Tw)

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        synchronize_and_publish()
        rate.sleep()

    # Rilascia il video writer e chiudi le finestre OpenCV
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
