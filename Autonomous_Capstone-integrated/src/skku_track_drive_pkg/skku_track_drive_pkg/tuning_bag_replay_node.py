"""Input-only rosbag playback, started after the target input subscription exists."""
import signal
import subprocess
import csv
import json
import time
import tempfile
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.executors import ExternalShutdownException
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


class TuningBagReplayNode(Node):
    def __init__(self):
        super().__init__('tuning_bag_replay_node')
        for key, value in [('bag_path',''),('input_topic','/track/image_raw'),('target_node','track_controller_node'),
                           ('mode','track'),('rate',1.0),('loop',False),('processed_csv',''),('wait_for_recorder',False)]:
            self.declare_parameter(key,value)
        self.process = None
        self.qos_directory = None
        self.done = Future()
        self.reader = None
        self.pending_stamp = None
        self.next_image = None
        self.first_stamp = self.first_wall = None
        processed = self.get_parameter('processed_csv').value
        if self.get_parameter('wait_for_recorder').value and not processed:
            # rosbag discovers types from publishers. Advertise the input type
            # without sending data so its subscription can exist before play.
            mode = self.get_parameter('mode').value
            topic = '/dry_run/vehicle/drive_state' if mode == 'calibration' else self.get_parameter('input_topic').value
            message_type = String if mode == 'calibration' else Image if mode in ('track','mission') else LaserScan
            self.input_advertiser = self.create_publisher(message_type,topic,10)
        if processed:
            if self.get_parameter('mode').value not in ('track','mission') or self.get_parameter('loop').value:
                raise ValueError('processed_csv is single-pass track/mission comparison only')
            if self.get_parameter('rate').value <= 0:
                raise ValueError('rate must be positive')
            import rosbag2_py
            with open(processed) as stream:
                self.stamps = {int(row['frame_stamp_ns']) for row in csv.DictReader(stream)}
            self.reader = rosbag2_py.SequentialReader()
            self.reader.open(rosbag2_py.StorageOptions(uri=self.get_parameter('bag_path').value,storage_id='sqlite3'),
                             rosbag2_py.ConverterOptions('',''))
            self.image_pub = self.create_publisher(Image,self.get_parameter('input_topic').value,1)
            self.create_subscription(String,'/'+self.get_parameter('target_node').value+'/perception_timing',self.ack,10)
        self.create_timer(.01,self.tick)

    def ack(self,message):
        if json.loads(message.data).get('frame_stamp_ns') == self.pending_stamp:
            self.pending_stamp = None

    def tick_processed(self):
        if self.pending_stamp is not None:
            return  # No additional vehicle timeout; Ctrl+C remains available.
        if self.next_image is None:
            while self.reader.has_next():
                topic,data,_ = self.reader.read_next()
                if topic != self.get_parameter('input_topic').value:
                    continue
                image = deserialize_message(data,Image)
                stamp = image.header.stamp.sec*1000000000+image.header.stamp.nanosec
                if stamp in self.stamps:
                    self.next_image = image
                    break
            if self.next_image is None:
                self.done.set_result(True)
                return
        image = self.next_image
        stamp = image.header.stamp.sec*1000000000+image.header.stamp.nanosec
        if self.first_stamp is None:
            self.first_stamp,self.first_wall = stamp,time.monotonic()
        if time.monotonic() < self.first_wall+(stamp-self.first_stamp)/1e9/self.get_parameter('rate').value:
            return
        self.pending_stamp = stamp
        self.image_pub.publish(image)
        self.next_image = None

    def tick(self):
        if self.process is not None:
            if self.process.poll() is not None:
                if self.process.returncode != 0:
                    raise RuntimeError(f'rosbag replay failed: {self.process.returncode}')
                self.done.set_result(True)
            return
        mode=self.get_parameter('mode').value
        topic=self.get_parameter('input_topic').value
        wait_topic='/dry_run/vehicle/drive_state' if mode == 'calibration' else topic
        target=self.get_parameter('target_node').value
        subscribers={s.node_name for s in self.get_subscriptions_info_by_topic(wait_topic)}
        if target not in subscribers:
            return
        if self.get_parameter('wait_for_recorder').value and 'rosbag2_recorder' not in subscribers:
            return
        if self.reader is not None:
            self.tick_processed()
            return
        states=[prefix+'/vehicle/drive_state' for prefix in ('','/sensors_only','/dry_run','/parking_calibration')]
        # Reliable offline inputs let the parallel bag recorder receive every
        # frame even if the production controller uses its latest-frame queue.
        self.qos_directory = tempfile.TemporaryDirectory(prefix='autodrive_replay_qos_')
        qos_file = Path(self.qos_directory.name)/'input_qos.yaml'
        qos_file.write_text(yaml.safe_dump({name:{'reliability':'reliable','history':'keep_last','depth':10}
            for name in [topic,*states,'/dry_run/vehicle/drive_state']}))
        cmd=['ros2','bag','play',self.get_parameter('bag_path').value,'--rate',str(self.get_parameter('rate').value),
             '--qos-profile-overrides-path',str(qos_file),
             '--disable-keyboard-controls','--topics',*([topic] if mode != 'calibration' else []),*states,
             '--remap',*(s+':=/dry_run/vehicle/drive_state' for s in states)]
        if self.get_parameter('loop').value:
            cmd.append('--loop')
        self.process=subprocess.Popen(cmd)

    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        if self.qos_directory is not None:
            self.qos_directory.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=TuningBagReplayNode()
        rclpy.spin_until_future_complete(node,node.done)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close();node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
