#!/usr/bin/env python

import rospy
from dvs_msgs.msg import EventArray
import numpy as np
import multiprocessing as mp
import os
from datetime import datetime

EV_TYPE = np.dtype([('t', 'u4'), ('_', 'i4')])

class BagToTxtNode:
    def __init__(self):
        self.events = []
        self.min_time = 0
        self.max_time = 0
        self.event_count = 0
        self.block_size = 100000  # Dimensione del blocco di memoria

        # Parametri ROS
        default_txt_dir = '/home/salvatore/ssms_event_cameras/dataset/'
        self.topic = rospy.get_param('~topic', '/dvs/events')
        self.txt_dir = rospy.get_param('~txt_dir', default_txt_dir)

        rospy.loginfo(f"Subscribing to topic: {self.topic}")
        rospy.loginfo(f"Output directory: {self.txt_dir}")

        # Sottoscrizione al topic
        rospy.Subscriber(self.topic, EventArray, self.callback)

        rospy.loginfo("BagToTxtNode initialized")

    def callback(self, msg):
        if len(self.events) == 0:
            self.min_time = msg.events[0].ts.to_nsec()

        for event in msg.events:
            current_time = event.ts.to_nsec()
            self.events.append([int((current_time - self.min_time) / 1e3), event.x, event.y, 0 if not event.polarity else 1])
            self.max_time = current_time  # Aggiorna l'ultimo timestamp
            self.event_count += 1

            if self.event_count >= self.block_size:
                self.handle_memory_block()
                self.event_count = 0

    def handle_memory_block(self):
        block = self.events.copy()
        last_timestamp = self.max_time
        self.events = []
        process = mp.Process(target=self.convert_events_to_dat, args=(block, self.file_count, 180, 240))  # Assumendo altezza e larghezza come 180x240, modifica se necessario
        process.start()
        self.file_count += 1
        rospy.loginfo(f"Generato nuovo processo per la conversione. Ultimo timestamp: {last_timestamp}")

    def convert_events_to_dat(self, events, file_count, height, width):
        dat_file = f"{self.txt_dir}{file_count}.dat"

        buffers = np.zeros(len(events), dtype=[('t', 'u4'), ('x', 'i2'), ('y', 'i2'), ('p', 'i2')])
        for i, event in enumerate(events):
            buffers['t'][i] = event[0]
            buffers['x'][i] = event[1]
            buffers['y'][i] = event[2]
            buffers['p'][i] = event[3]

        with open(dat_file, 'wb') as f:
            self.write_header(f, height, width)
            self.write_event_buffer(f, buffers)

    def write_header(self, f, height, width, ev_type=0):
        header = (
            f"% Data file containing Event2D events.\n"
            f"% Version 2\n"
            f"% Date {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
            f"% Height {height}\n"
            f"% Width {width}\n"
        )
        f.write(header.encode('latin-1'))
        ev_size = np.array([ev_type, 8], dtype=np.uint8)
        ev_size.tofile(f)

    def write_event_buffer(self, f, buffers):
        dtype = EV_TYPE
        data_to_write = np.empty(len(buffers), dtype=dtype)

        x = buffers['x'].astype('i4')
        y = np.left_shift(buffers['y'].astype('i4'), 14)
        p = np.left_shift(buffers['p'].astype('i4'), 28)

        data_to_write['_'] = x + y + p
        data_to_write['t'] = buffers['t']

        data_to_write.tofile(f)
        f.flush()

    def shutdown_hook(self):
        rospy.loginfo("Shutting down BagToTxtNode")
        if self.events:
            self.handle_memory_block()

if __name__ == '__main__':
    rospy.init_node('bag_to_txt_node', anonymous=True)
    node = BagToTxtNode()
    rospy.on_shutdown(node.shutdown_hook)
    rospy.spin()
