import os
import numpy as np
# ros2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# depth_anything_v2
import cv2
import torch
from mono_depth.DepthAnythingV2.depth_anything_v2.dpt import DepthAnythingV2

class MonoDepth(Node):
	MIN_DEPTH = 1
	MAX_DEPTH = 10
	DEPTH_MULTIPLIER = 1
	# ノード名
	SELFNODE = "mono_depth"
	MODEL_PAHT = os.path.abspath(__file__.replace('mono_depth.py', 'models'))
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
		# モデルの読み込み
		self.get_logger().info("model path:%s " % (self.MODEL_PAHT))
		self.model = DepthAnythingV2(**self.model_configs[self.encoder])
		self.model.load_state_dict(torch.load(self.MODEL_PAHT+f'/depth_anything_v2_{self.encoder}.pth', map_location='cpu'))
		self.model = self.model.to(self.DEVICE).eval()
		self.get_logger().info("Using %s" % (self.DEVICE))
		# ros2 init
		self.image_pub_ = self.create_publisher(Image,'mono_depth/depth', 1)
		self.info_pub_ = self.create_publisher(CameraInfo,'mono_depth/camera_info', 1)
		self.image_sub_ = self.create_subscription(Image,'/thetav/image_raw', self.image_callback, qos_profile=ReliabilityPolicy.RELIABLE)
		self.info_sub_ = self.create_subscription(CameraInfo,'/thetav/camera_info', self.info_callback, qos_profile=ReliabilityPolicy.RELIABLE)
		self.image_ = None
		self.info_ = None
		self.bridge_ = CvBridge()
		self.frame_id = 'camera_depth_optical_frame'

	def __del__(self):
		self.get_logger().info("%s done." % self.SELFNODE)

	def inversion(self,depth):
		ratio = (depth - depth.min()) / (depth.max() - depth.min())
		depth_scaled = self.MIN_DEPTH + (self.MAX_DEPTH - self.MIN_DEPTH) * ratio
		depth = (self.DEPTH_MULTIPLIER / depth_scaled) * 65535
		depth = depth.astype(np.uint16)
		return depth

	def image_callback(self, msg):
		# print("image_callback")
		self.image_ = msg
		# if self.image_ is None:
		# 	return
		raw_image = self.bridge_.imgmsg_to_cv2(self.image_)
		raw_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2RGB)
		depth = self.model.infer_image(raw_image)
		depth = self.inversion(depth)
		img_msg = self.bridge_.cv2_to_imgmsg(depth, encoding='mono16')
		img_msg.header = self.image_.header
		# img_msg.header.frame_id = self.frame_id
		if self.info_ is not None:
			self.info_.header = self.image_.header
			self.image_pub_.publish(img_msg)
			self.info_pub_.publish(self.info_)
		else:
			self.get_logger().warn("info is None")
			self.image_pub_.publish(img_msg)

	def info_callback(self, msg):
		# print("info_callback")
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