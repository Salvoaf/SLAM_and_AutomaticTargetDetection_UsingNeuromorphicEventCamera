#!/usr/bin/env python

import rospy
from dvs_msgs.msg import EventArray
import numpy as np
import multiprocessing as mp
import os
from datetime import datetime
import h5py
import sys

EV_TYPE = [("t", "u4"), ("_", "i4")]  # Event2D

class BagToH5Node:
    def __init__(self):
        self.events = []
        self.min_time = 0
        self.max_time = 0
        self.event_count = 0
        self.block_size = 250000  # Memory block size
        self.file_count = 0  # File counter initialization

        # ROS parameters
        default_txt_dir = '/home/salvatore/ssms_event_cameras/dataset/val/'
        self.topic = rospy.get_param('~topic', '/dvs/events')
        self.txt_dir = rospy.get_param('~txt_dir', default_txt_dir)

        rospy.loginfo(f"Subscribing to topic: {self.topic}")
        rospy.loginfo(f"Output directory: {self.txt_dir}")

        # Subscribe to the topic
        rospy.Subscriber(self.topic, EventArray, self.callback)

        rospy.loginfo("BagToH5Node initialized")

    def callback(self, msg):
        if len(self.events) == 0:
            self.min_time = msg.events[0].ts.to_nsec()

        for event in msg.events:
            current_time = event.ts.to_nsec()
            self.events.append([int((current_time - self.min_time) / 1e3), event.x, event.y, 0 if not event.polarity else 1])
            self.max_time = current_time  # Update the last timestamp
            self.event_count += 1

            if self.event_count >= self.block_size:
                self.handle_memory_block()
                self.event_count = 0

    def handle_memory_block(self):
        block = self.events.copy()
        last_timestamp = self.max_time
        self.events = []
        process = mp.Process(target=self.convert_events_to_h5, args=(block, self.file_count, 180, 240))  # Assuming height and width as 180x240, modify if necessary
        process.start()
        self.file_count += 1
        rospy.loginfo(f"Started new process for conversion. Last timestamp: {last_timestamp}")

    def convert_events_to_h5(self, events, file_count, height, width):
        dat_file = os.path.join(self.txt_dir, f"{file_count}.dat")
        h5_file = os.path.join(self.txt_dir, f"{file_count}.dat.h5")
        npy_file = os.path.join(self.txt_dir, f"{file_count}.npy")
        first_timestamp = 0
        last_timestamp = 0

        buffers = np.zeros(len(events), dtype=[('t', 'u4'), ('x', 'i2'), ('y', 'i2'), ('p', 'i2')])
        for i, event in enumerate(events):
            if i == 0:
                first_timestamp = event[0]
            last_timestamp = event[0]
            buffers['t'][i] = event[0]
            buffers['x'][i] = event[1]
            buffers['y'][i] = event[2]
            buffers['p'][i] = event[3]

        self.create_npy(npy_file, last_timestamp, first_timestamp, 5000, height, width)

        with open(dat_file, 'wb') as f:
            self.write_header(f, height, width)
            self.write_event_buffer(f, buffers)

        convert_dat_to_h5(dat_file, h5_file, height, width)

        os.remove(dat_file)

    @staticmethod
    def create_npy(npy_file, max_ts, initial_timestamp, jump, height, width):
        num_rows = int((max_ts - initial_timestamp) / jump)

        new_timestamps = np.array([initial_timestamp + i * jump for i in range(num_rows)], dtype=np.uint64)
        x_values = np.random.randint(0, width, size=num_rows, dtype=np.int32).astype(np.float32)
        y_values = np.random.randint(0, height, size=num_rows, dtype=np.int32).astype(np.float32)
        w_values = np.random.randint(2, 61, size=num_rows, dtype=np.int32).astype(np.float32)
        h_values = np.random.randint(3, 71, size=num_rows, dtype=np.int32).astype(np.float32)
        class_id_values = np.random.randint(0, 2, size=num_rows, dtype=np.uint8)
        confidence_values = np.ones(num_rows, dtype=np.float32)
        track_id_values = np.random.randint(0, 2, size=num_rows, dtype=np.uint32)

        new_data = np.zeros(num_rows, dtype=[('ts', '<u8'), ('x', '<f4'), ('y', '<f4'), ('w', '<f4'), ('h', '<f4'), ('class_id', 'u1'), ('confidence', '<f4'), ('track_id', '<u4')])
        new_data['ts'] = new_timestamps
        new_data['x'] = x_values
        new_data['y'] = y_values
        new_data['w'] = w_values
        new_data['h'] = h_values
        new_data['class_id'] = class_id_values
        new_data['confidence'] = confidence_values
        new_data['track_id'] = track_id_values

        np.save(npy_file, new_data)

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
        rospy.loginfo("Shutting down BagToH5Node")
        if self.events:
            self.handle_memory_block()

def load_td_data(filename, ev_count=-1, ev_start=0):
    with open(filename, "rb") as f:
        _, ev_type, ev_size, _ = parse_header(f)
        if ev_start > 0:
            f.seek(ev_start * ev_size, 1)
        dtype = EV_TYPE
        dat = np.fromfile(f, dtype=dtype, count=ev_count)
        xyp = None
        if ("_", "i4") in dtype:
            x = np.bitwise_and(dat["_"], 16383)
            y = np.right_shift(np.bitwise_and(dat["_"], 268419072), 14)
            p = np.right_shift(np.bitwise_and(dat["_"], 268435456), 28)
            xyp = (x, y, p)
        return dat_transfer(dat, dtype, xyp=xyp)

def dat_transfer(dat, dtype, xyp=None):
    variables = []
    xyp_index = -1
    for i, (name, _) in enumerate(dtype):
        if name == "_":
            xyp_index = i
            continue
        variables.append((name, dat[name]))
    if xyp and xyp_index == -1:
        print("Error dat didn't contain a '_' field !")
        return None
    if xyp_index >= 0:
        dtype = (
            dtype[:xyp_index]
            + [("x", "i2"), ("y", "i2"), ("p", "i2")]
            + dtype[xyp_index + 1 :]
        )
    new_dat = np.empty(dat.shape[0], dtype=dtype)
    if xyp:
        new_dat["x"] = xyp[0].astype(np.uint16)
        new_dat["y"] = xyp[1].astype(np.uint16)
        new_dat["p"] = xyp[2].astype(np.uint16)
    for name, arr in variables:
        new_dat[name] = arr
    return new_dat

def parse_header(f):
    f.seek(0, os.SEEK_SET)
    bod = None
    end_of_header = False
    header = []
    num_comment_line = 0
    size = [None, None]
    while not end_of_header:
        bod = f.tell()
        line = f.readline()
        if sys.version_info > (3, 0):
            first_item = line.decode("latin-1")[:2]
        else:
            first_item = line[:2]
        if first_item != "% ":
            end_of_header = True
        else:
            words = line.split()
            if len(words) > 1:
                if words[1] == "Date":
                    header += ["Date", words[2] + " " + words[3]]
                if words[1] == "Height" or words[1] == b"Height":
                    size[0] = int(words[2])
                    header += ["Height", words[2]]
                if words[1] == "Width" or words[1] == b"Width":
                    size[1] = int(words[2])
                    header += ["Width", words[2]]
            else:
                header += words[1:3]
            num_comment_line += 1
    f.seek(bod, os.SEEK_SET)
    if num_comment_line > 0:
        ev_type = np.frombuffer(f.read(1), dtype=np.uint8)[0]
        ev_size = np.frombuffer(f.read(1), dtype=np.uint8)[0]
    else:
        ev_type = 0
        ev_size = sum([int(n[-1]) for _, n in EV_TYPE])
    bod = f.tell()
    return bod, ev_type, ev_size, size

def convert_dat_to_h5(dat_file_path, h5_file_path, height, width):
    try:
        events = load_td_data(dat_file_path)
        if events is None:
            print("Error during data transfer")
            return

        required_fields = ['t', 'x', 'y', 'p']
        for field in required_fields:
            if field not in events.dtype.names:
                raise ValueError(f"The field '{field}' is not present in the loaded data")

        with h5py.File(h5_file_path, 'w') as h5_file:
            events_group = h5_file.create_group('events')
            events_group.create_dataset('t', data=events['t'])
            events_group.create_dataset('x', data=events['x'])
            events_group.create_dataset('y', data=events['y'])
            events_group.create_dataset('p', data=events['p'])
            events_group.attrs['height'] = height
            events_group.attrs['width'] = width

        if os.path.exists(h5_file_path):
            print(f"Conversion successfully completed: {h5_file_path}")
        else:
            print(f"Error: the file {h5_file_path} was not created correctly.")

    except Exception as e:
        print(f"Error during conversion: {e}")

if __name__ == '__main__':
    print("aaaaaaaaaaaaaaaa")
    rospy.init_node('bag_to_h5_node', anonymous=True, disable_signals=True)
    print("bbbbbbbbbbb")
    node = BagToH5Node()
    rospy.on_shutdown(node.shutdown_hook)
    rospy.spin()
