import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class MonoDepth(Node):
  # ノード名
  SELFNODE = "mono_depth"
  def __init__(self):
    # ノードの初期化
    super().__init__(self.SELFNODE)
    self.get_logger().info("%s initializing..." % (self.SELFNODE))
    self.image_pub_ = self.create_publisher(Image,'mono_depth/depth', 10)
    self.info_pub_ = self.create_publisher(CameraInfo,'mono_depth/depth_info', 10)
    self.image_sub_ = self.create_subscription(Image,'/camera/sp360_4k/pano/image_expand', self.image_callback, 10)
    self.info_sub_ = self.create_subscription(CameraInfo,'/camera/sp360_4k/pano/camera_info', self.info_callback, 10)
    self.create_timer(0.01, self.callback)
    self.image_ = None
    self.info_ = None

  def __del__(self):
    self.get_logger().info("%s done." % self.SELFNODE)

  def image_callback(self, msg):
    self.image_ = msg

  def info_callback(self, msg):
    self.info_ = msg

  def callback(self):
    if self.image_ is None or self.info_ is None:
      return
    self.image_.header.stamp = self.get_clock().now().to_msg()
    self.info_.header.stamp = self.get_clock().now().to_msg()

    self.image_pub_.publish(self.image_)
    self.info_pub_.publish(self.info_)


def main(args=None):
  try:
    # rclpyの初期化
    rclpy.init(args=args)
    # インスタンスを生成
    node=MonoDepth()
    # プロセス終了までアイドリング
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    # 終了処理
    rclpy.shutdown()


if __name__ == '__main__':
  main()