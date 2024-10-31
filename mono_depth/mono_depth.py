import os
import numpy as np
import argparse
# ros2
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy import time
from std_msgs.msg import Header, Float32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# depth_anything_v2
import cv2
import torch
from mono_depth.DepthAnythingV2.depth_anything_v2.dpt import DepthAnythingV2
from mono_depth.DepthAnythingV2.depth_anything_v2.util.transform import NormalizeImage, PrepareForNet, Resize
from mono_depth.DepthAnythingV2.depth_anything_v2.dpt_trt import DepthAnythingV2 as DepthAnythingV2TRT
import pycuda.driver as cuda
import pycuda.autoinit
import tensorrt as trt
from torchvision.transforms import Compose

class MonoDepth(Node):
  # デプス関連パラメータ
  MIN_DEPTH = 1
  MAX_DEPTH = 10
  D_DEPTH = MAX_DEPTH - MIN_DEPTH
  # ノード名
  SELFNODE = "mono_depth"
  # model path
  MODEL_PAHT = os.path.abspath(__file__.replace('mono_depth.py', 'models'))
  # device setting
  DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

  model_configs = {
      'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
      'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
      'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
      'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
  }

  encoder = 'vits' # or 'vits', 'vitb', 'vitg'

  def __init__(self):
    # ノードの初期化
    super().__init__(self.SELFNODE)
    self.get_logger().info("%s initializing..." % (self.SELFNODE))
    # パラメータの取得
    self.USE_TRT = self.param("use_trt", True).bool_value
    self.encoder =  self.param("encoder", 'vits').string_value # 'vits', 'vitb', 'vitl'
    # モデルの読み込み
    self.get_logger().info("model path:%s " % (self.MODEL_PAHT))
    self.model = DepthAnythingV2(**self.model_configs[self.encoder])
    # self.model = DepthAnythingV2TRT(**self.model_configs[self.encoder])
    self.model.load_state_dict(torch.load(self.MODEL_PAHT+f'/depth_anything_v2_{self.encoder}.pth', map_location='cpu'))
    # TensorRT
    if self.USE_TRT:
      self.ctx = cuda.Device(0).make_context()
      self.logger = trt.Logger(trt.Logger.WARNING)
      with open(self.MODEL_PAHT+f'/depth_anything_v2_{self.encoder}.engine', 'rb') as f, trt.Runtime(self.logger) as runtime:
        self.engine = runtime.deserialize_cuda_engine(f.read())
      self.get_logger().info("Using TensorRT")
    self.model = self.model.to(self.DEVICE).eval()
    self.get_logger().info("Using %s" % (self.DEVICE))
    # publisher
    self.image_pub_ = self.create_publisher(Image,'mono_depth/depth', 1)
    self.info_pub_ = self.create_publisher(CameraInfo,'mono_depth/camera_info', 1)
    self.calc_time_pub_ = self.create_publisher(Float32,'mono_depth/calc_time', 1)
    # subscriber
    self.image_sub_ = self.create_subscription(Image,'image_raw', self.image_callback, qos_profile=ReliabilityPolicy.RELIABLE)
    self.info_sub_ = self.create_subscription(CameraInfo,'camera_info', self.info_callback, qos_profile=ReliabilityPolicy.RELIABLE)
    # 初期化
    self.image_ = None
    self.info_ = None
    self.bridge_ = CvBridge()

  def __del__(self):
    self.get_logger().info("%s done." % self.SELFNODE)

  def param(self, name, value):
    self.declare_parameter(name, value)
    return self.get_parameter(name).get_parameter_value()

  def tf_range(self,value, in_min, in_max):
    return (value - in_min) * self.D_DEPTH / (in_max - in_min) + self.MIN_DEPTH

  def inversion(self,depth):
    tf_depth = self.tf_range(depth, depth.min(), depth.max())
    depth = (65535.0 / tf_depth).astype(np.uint16)
    return depth

  transform = Compose(
    [
      Resize(
        # width=w,
        # height=h,
        width=518,
        height=518,
        resize_target=False,
        keep_aspect_ratio=False,
        ensure_multiple_of=14,
        resize_method="lower_bound",
        image_interpolation_method=cv2.INTER_CUBIC,
      ),
      NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
      PrepareForNet(),
    ]
  )

  def infer_image(self,raw_image):
    self.ctx.push()
    try:
      h, w = raw_image.shape[:2]
      raw_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2RGB) / 255.0
      input_image = self.transform({'image': raw_image})['image']
      input_image = input_image[None]
      # Create logger and load the TensorRT engine
      with self.engine.create_execution_context() as context:
        input_shape = context.get_tensor_shape('input')
        output_shape = context.get_tensor_shape('output')
        h_input = cuda.pagelocked_empty(trt.volume(input_shape), dtype=np.float32)
        h_output = cuda.pagelocked_empty(trt.volume(output_shape), dtype=np.float32)
        d_input = cuda.mem_alloc(h_input.nbytes)
        d_output = cuda.mem_alloc(h_output.nbytes)
        stream = cuda.Stream()
        # Copy the input image to the pagelocked memory
        np.copyto(h_input, input_image.ravel())
        # Copy the input to the GPU, execute the inference, and copy the output back to the CPU
        cuda.memcpy_htod_async(d_input, h_input, stream)
        context.execute_async_v2(bindings=[int(d_input), int(d_output)], stream_handle=stream.handle)
        cuda.memcpy_dtoh_async(h_output, d_output, stream)
        stream.synchronize()
        depth = h_output
      # Process the depth output
      depth = np.reshape(depth, (518, 518))
      depth = cv2.resize(depth, (w, h))
    finally:
      self.ctx.pop() # Ensure the context is popped at the end of the function
    return depth

  def image_callback(self, msg):
    start_time = Clock(clock_type=ClockType.ROS_TIME).now()
    # openCV形式に変換
    self.image_ = msg
    raw_image = self.bridge_.imgmsg_to_cv2(self.image_)
    raw_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2RGB)
    # 深度推定
    if self.USE_TRT:
      depth = self.infer_image(raw_image)
    else:
      depth = self.model.infer_image(raw_image)
    # 深度反転
    depth = self.inversion(depth)
    # publish
    img_msg = self.bridge_.cv2_to_imgmsg(depth, encoding='mono16')
    img_msg.header = self.image_.header
    if self.info_ is not None:
      self.info_.header = self.image_.header
      self.image_pub_.publish(img_msg)
      self.info_pub_.publish(self.info_)
    else:
      self.get_logger().warn("info is None")
      self.image_pub_.publish(img_msg)
    # 処理時間
    durtion = (Clock(clock_type=ClockType.ROS_TIME).now() - start_time).nanoseconds / 1000000000
    self.get_logger().info("time:%f" % durtion)
    self.calc_time_pub_.publish(Float32(data=durtion))

  def info_callback(self, msg):
    self.info_ = msg

def main(args=None):
  try:
    rclpy.init(args=args)
    node=MonoDepth()
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    # 終了処理
    rclpy.shutdown()

if __name__ == '__main__':
  main()