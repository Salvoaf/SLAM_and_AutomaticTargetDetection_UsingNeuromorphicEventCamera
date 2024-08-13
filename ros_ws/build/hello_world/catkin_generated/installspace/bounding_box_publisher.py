#!/usr/bin/env python3

import rospy
import csv
import os
import time
from hello_world.msg import BoundingBoxWithTime 

def numpy_to_dict(np_array, fieldnames):
    np_array = {fieldname: value for fieldname, value in zip(fieldnames, np_array)}
    return {
        't': np.int64(np_array['t']),
        'x': np.int32(np_array['x']),
        'y': np.int32(np_array['y']),
        'w': np.int32(np_array['w']),
        'h': np.int32(np_array['h']),
        'class_id': np.int32(np_array['class_id']),
        'class_confidence': np.float32(np_array['class_confidence']),
        'track_id': np.int32(np_array['track_id'])
    }

def read_and_publish(predictions_file_path, pred_fieldnames):
    rospy.init_node('bounding_box_publisher', anonymous=True)
    rospy.loginfo("Nodo inizializzato")
    pub = rospy.Publisher('bounding_box_with_time', BoundingBoxWithTime, queue_size=10)
    
    while not rospy.is_shutdown():
        if os.path.isfile(predictions_file_path):
            temp_file_path = predictions_file_path.replace('predictions.csv', 'pred.csv')
            os.rename(predictions_file_path, temp_file_path)
            rospy.loginfo(f"File rinominato: {temp_file_path}")

            with open(temp_file_path, 'r') as csvfile:
                reader = csv.DictReader(csvfile, fieldnames=pred_fieldnames)
                next(reader)  # Salta l'intestazione
                for row in reader:
                    bbox_with_time = BoundingBoxWithTime()
                    bbox_with_time.timestamp = float(row['t']) / 1e6  # Converte il timestamp in secondi
                    bbox_with_time.x_offset = int(row['x'])
                    bbox_with_time.y_offset = int(row['y'])
                    bbox_with_time.height = int(row['h'])
                    bbox_with_time.width = int(row['w'])
                    bbox_with_time.do_rectify = False  # Se l'immagine deve essere rettificata
                    bbox_with_time.class_id = int(row['class_id'])
                    bbox_with_time.class_confidence = float(row['class_confidence'])
                    bbox_with_time.track_id = int(row['track_id'])
                    
                    pub.publish(bbox_with_time)
                    rospy.loginfo(f"Bounding box con timestamp pubblicata: {bbox_with_time}")
                
            # Elimina il file dopo averlo letto
            os.remove(temp_file_path)
            rospy.loginfo(f"File eliminato: {temp_file_path}")
        else:
            rospy.loginfo(f"File non trovato, attendo: {predictions_file_path}")
            time.sleep(1)  # Attende prima di controllare nuovamente

    rospy.signal_shutdown('Finished publishing bounding boxes')

if __name__ == '__main__':
    try:
        pred_fieldnames = ['t', 'x', 'y', 'w', 'h', 'class_id', 'class_confidence', 'track_id']
        predictions_file_path = rospy.get_param('~predictions_file_path', '/home/salvatore/ssms_event_cameras/poses/predictions.csv')
        rospy.loginfo(f"Percorso del file delle predizioni: {predictions_file_path}")
        read_and_publish(predictions_file_path, pred_fieldnames)
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"ROS Interrupt: {e}")
    except Exception as e:
        rospy.logerr(f"Errore durante l'esecuzione del nodo: {e}")
