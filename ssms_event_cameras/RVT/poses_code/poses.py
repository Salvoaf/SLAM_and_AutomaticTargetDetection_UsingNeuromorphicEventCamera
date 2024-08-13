import rclpy
import sys
import numpy as np
import csv
from tqdm import tqdm
import os

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros.transform_broadcaster import TransformBroadcaster

class DatasetBroadcaster(Node):
    def __init__(self, dataset_path, output_path):
        super().__init__('dataset_broadcaster')
        self.get_logger().info("Broadcasting poses from dataset: {}".format(dataset_path))
        self.tfb_ = TransformBroadcaster(self)
        
        # Definizione del dtype corretto
        self.desired_dtype = np.dtype([
            ('ts', '<u8'), 
            ('x', '<f4'), 
            ('y', '<f4'), 
            ('w', '<f4'), 
            ('h', '<f4'), 
            ('class_id', 'u1'), 
            ('class_confidence', '<f4'), 
            ('track_id', '<u4')
        ])
        
        # Lettura del dataset
        self.dataset = np.genfromtxt(dataset_path, delimiter=',', dtype=self.desired_dtype, skip_header=1)

        # Output file for saving poses
        self.output_file = output_path
        with open(self.output_file, mode='w') as file:
            writer = csv.writer(file, delimiter='\t')
            writer.writerow(['frame_id', 'child_frame_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

        # Inizializzazione della barra di progresso
        self.pbar = tqdm(total=len(self.dataset), desc='Broadcasting poses', unit='pose')
        
        # Variabile per tracciare l'indice corrente
        self.current_index = 0
        
        self.timer = self.create_timer(0.1, self.broadcast_transforms)

    def broadcast_transforms(self):
        if self.current_index < len(self.dataset):
            row = self.dataset[self.current_index]
            tfs = TransformStamped()
            tfs.header.stamp = self.get_clock().now().to_msg()
            tfs.header.frame_id = "world"
            tfs.child_frame_id = "object_{}_class_{}".format(self.current_index, row['class_id'])
            tfs.transform.translation.x = float(row['x'])
            tfs.transform.translation.y = float(row['y'])
            tfs.transform.translation.z = 0.0
            
            r = R.from_euler('xyz', [0, 0, 0])  # Assuming no rotation information
            tfs.transform.rotation.x = r.as_quat()[0]
            tfs.transform.rotation.y = r.as_quat()[1]
            tfs.transform.rotation.z = r.as_quat()[2]
            tfs.transform.rotation.w = r.as_quat()[3]

            self.tfb_.sendTransform(tfs)

            # Salva la posa nel file
            with open(self.output_file, mode='a') as file:
                writer = csv.writer(file, delimiter='\t')
                writer.writerow([
                    tfs.header.frame_id,
                    tfs.child_frame_id,
                    tfs.transform.translation.x,
                    tfs.transform.translation.y,
                    tfs.transform.translation.z,
                    tfs.transform.rotation.x,
                    tfs.transform.rotation.y,
                    tfs.transform.rotation.z,
                    tfs.transform.rotation.w
                ])
            
            # Aggiorna la barra di progresso
            self.pbar.update(1)
            self.current_index += 1
        else:
            # Chiudi la barra di progresso dopo il completamento
            self.pbar.close()
            self.get_logger().info("Completed broadcasting all poses.")
            os._exit(0)

def main():
    dataset_path = '/home/salvatore/ssms_event_cameras/poses/predictions.csv'
    output_path = '/home/salvatore/ssms_event_cameras/poses/poses.txt'
    
    rclpy.init(args=None)
    node = DatasetBroadcaster(dataset_path, output_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
