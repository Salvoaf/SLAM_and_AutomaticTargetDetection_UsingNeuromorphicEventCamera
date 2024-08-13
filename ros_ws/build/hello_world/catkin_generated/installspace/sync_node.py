import rospy
from sensor_msgs.msg import Image
from hello_world.msg import BoundingBoxWithTime  # Importa il messaggio custom corretto
from cv_bridge import CvBridge
import cv2
from collections import deque

# Buffer per le immagini e le bounding box
image_buffer = deque()
bbox_buffer = deque()
max_delay = 30  # Massimo delay in secondi

bridge = CvBridge()

# Inizializza il video writer
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))  # Specifica il nome del file, codec, fps e dimensione frame

def image_callback(image_msg):
    # Aggiungi l'immagine al buffer con il suo timestamp
    image_buffer.append((image_msg.header.stamp.to_sec(), image_msg))

def bbox_callback(bbox_msg):
    # Aggiungi la bounding box al buffer con il suo timestamp
    bbox_buffer.append((bbox_msg.timestamp, bbox_msg))

    # Rimuovi vecchie immagini dal buffer
    while image_buffer and image_buffer[0][0] < bbox_msg.timestamp - max_delay:
        image_buffer.popleft()

    # Cerca l'immagine con il timestamp più vicino alla bounding box
    closest_image = min(image_buffer, key=lambda x: abs(x[0] - bbox_msg.timestamp), default=None)
    
    if closest_image:
        synchronize(closest_image[1], bbox_msg)

def synchronize(image_msg, bbox_msg):
    # Converti l'immagine da ROS Image a OpenCV
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

    # Disegna la bounding box sull'immagine
    x_offset = bbox_msg.x_offset
    y_offset = bbox_msg.y_offset
    width = bbox_msg.width
    height = bbox_msg.height

    top_left = (x_offset, y_offset)
    bottom_right = (x_offset + width, y_offset + height)

    color = (0, 255, 0)  # Verde per la bounding box
    thickness = 2

    cv2.rectangle(cv_image, top_left, bottom_right, color, thickness)

    # Aggiungi informazioni sulla classe e la confidenza
    class_id = bbox_msg.class_id
    class_confidence = bbox_msg.class_confidence
    text = f"ID: {class_id}, Conf: {class_confidence:.2f}"
    cv2.putText(cv_image, text, (x_offset, y_offset - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)

    # Scrivi il frame nel video
    out.write(cv_image)

    # Visualizza l'immagine con la bounding box
    cv2.imshow('Synchronized Image', cv_image)
    cv2.waitKey(1)

def main():
    rospy.init_node('sync_node')
    rospy.Subscriber('/ze_vio/event_img', Image, image_callback)
    rospy.Subscriber('/bounding_box_publisher', BoundingBoxWithTime, bbox_callback)
    rospy.spin()

    # Rilascia il video writer e chiudi le finestre OpenCV
    out.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
