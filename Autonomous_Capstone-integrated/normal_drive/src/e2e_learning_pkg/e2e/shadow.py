"""ROS shadow prediction only. No MotionCommand publisher, serial access or arm service."""
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
from interfaces_pkg.msg import SourceFrame
import torch
from .learning import load_checkpoint,preprocess

class Shadow(Node):
    def __init__(self):
        super().__init__('e2e_shadow')
        self.declare_parameter('checkpoint','');self.declare_parameter('device','cpu')
        self.device=self.get_parameter('device').value
        torch.set_num_threads(2)
        self.model,self.meta=load_checkpoint(self.get_parameter('checkpoint').value,self.device)
        self.pub=self.create_publisher(String,'/e2e/prediction',10)
        self.latest=None;self.lock=threading.Lock();self.done=threading.Event()
        self.sub=self.create_subscription(SourceFrame,'/e2e/source_frame',self.receive,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.worker=threading.Thread(target=self.predict,daemon=True);self.worker.start()
    def receive(self,m):
        with self.lock:self.latest=m
    def predict(self):
        while not self.done.wait(.001):
            with self.lock:m,self.latest=self.latest,None
            if m is None:continue
            try:
                im=m.image
                if im.encoding!='bgr8':raise ValueError('Expected bgr8')
                frame=np.frombuffer(im.data,dtype=np.uint8).reshape(im.height,im.step)[:,:im.width*3].reshape(im.height,im.width,3)
                started=time.perf_counter_ns()
                with torch.inference_mode():value=float(self.model(torch.from_numpy(preprocess(frame,self.meta['preprocess'])).unsqueeze(0).to(self.device)).item())
                result=dict(shadow=True,camera_instance=m.camera_instance,source_sequence=m.sequence,
                    source_stamp_ns=int(im.header.stamp.sec)*1000000000+int(im.header.stamp.nanosec),source_clock_domain=m.clock_domain,
                    prediction_normalized=value,prediction_steps=value*7,command_step_if_selected=int(round(value*7)),
                    inference_pipeline_ns=time.perf_counter_ns()-started,generated_ros_ns=self.get_clock().now().nanoseconds,
                    dataset_version=self.meta['dataset_version'],actuation_enabled=False)
                self.pub.publish(String(data=json.dumps(result)))
            except Exception as e:self.get_logger().error(str(e))
    def destroy_node(self):
        self.done.set();self.worker.join(timeout=5);super().destroy_node()

def main(args=None):
    rclpy.init(args=args);node=None
    try:node=Shadow();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
