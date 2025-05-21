# 石の座標をサブスクライブしてMarkerをパブリッシュ
# python3 stone_rviz.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
#from builtin_interfaces.msg import Duration
#from std_msgs.msg import Header

class StoneMarkerPublisher(Node):
    def __init__(self):
        super().__init__('stone_marker_publisher')
        self.sub = self.create_subscription(Point, 'stone_xyz', self.callback, 10)
        self.pub_stone = self.create_publisher(Marker, 'stone_marker', 10)
        self.pub_stone_point = self.create_publisher(Marker, 'stone_point_marker', 10)
        self.pub_axis_labels = self.create_publisher(Marker, 'axis_labels_marker', 10)
        self.pub_camera_arrow = self.create_publisher(Marker, 'camera_arrow_marker', 10)

        # タイマーで軸ラベルを定期的に表示
        self.create_timer(1.0, self.publish_axis_labels)
        # タイマーでカメラの向きの矢印を定期的に表示
        self.create_timer(1.0, self.publish_camera_arrow)

    def callback(self, msg):
        # 石のマーカーを表示
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "stone"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        #marker.lifetime = Duration(sec=1)
        marker.pose.position = msg
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.pub_stone.publish(marker)

        # 石のマーカーの上に座標を表示
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "stone_point"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        #marker.lifetime = Duration(sec=1)
        marker.pose.position.x = msg.x
        marker.pose.position.y = msg.y
        marker.pose.position.z = msg.z + 0.1
        #marker.scale.x = 0.1  # 文字の幅(無効？)
        #marker.scale.y = 0.1  # 文字の高さ(無効？)
        marker.scale.z = 0.1  # 文字のサイズ
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"(x={msg.x:.2f},y={msg.y:.2f},z={msg.z:.2f})"
        self.pub_stone_point.publish(marker)

    # 原点の座標軸にX,Y,Zのラベルを表示
    def publish_axis_labels(self):
        labels = [
            ("X", 1.0, 0.0, 0.0, 0, (1.0, 0.0, 0.0)),
            ("Y", 0.0, 1.0, 0.0, 1, (0.0, 1.0, 0.0)),
            ("Z", 0.0, 0.0, 1.0, 2, (0.0, 0.0, 1.0)),
        ]
        for label, x, y, z, id, (r, g, b) in labels:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "axis_labels"
            marker.id = id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            #marker.scale.x = 0.1  # 文字の幅(無効？)
            #marker.scale.y = 0.1  # 文字の高さ(無効？)
            marker.scale.z = 0.3  # 文字のサイズ
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0
            marker.text = label
            self.pub_axis_labels.publish(marker)

    # 原点にカメラの向きの矢印を表示
    def publish_camera_arrow(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "camera_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        #marker.lifetime = Duration(sec=1)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = 0.7
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 1.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.3
        self.pub_camera_arrow.publish(marker)

# 初期化
rclpy.init()

# クラスをインスタンス化
stone_marker_publisher = StoneMarkerPublisher()

# spin()でループ処理を実行
rclpy.spin(stone_marker_publisher)

# 終了
stone_marker_publisher.destroy_node()
rclpy.shutdown()