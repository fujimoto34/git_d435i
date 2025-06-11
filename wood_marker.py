# 木材の座標をサブスクライブして両端を結ぶ線をパブリッシュ
# python3 wood_marker.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
#from builtin_interfaces.msg import Duration
#from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped  # 2点同時パブリッシュに使うPoint型
from message_filters import Subscriber, TimeSynchronizer  # トピックの同期に必要


class WoodMarkerPublisher(Node):
    def __init__(self):
        super().__init__('wood_marker_publisher')
        self.sub_A = Subscriber(self, PointStamped, 'wood_xyz_A')  # create_subscriptionではなくこちらを使う。引数も変わるので注意。
        self.sub_B = Subscriber(self, PointStamped, 'wood_xyz_B')
        self.sync = TimeSynchronizer([self.sub_A, self.sub_B], queue_size=10)  # 2つのトピックを同期
        self.sync.registerCallback(self.callback)  # 2つのトピックが揃ってからcallback関数を呼ぶ
        self.pub_wood = self.create_publisher(Marker, 'wood_marker', 10)
        self.pub_wood_point = self.create_publisher(Marker, 'wood_point_marker', 10)
        self.pub_axis_labels = self.create_publisher(Marker, 'axis_labels_marker', 10)
        self.pub_camera_arrow = self.create_publisher(Marker, 'camera_arrow_marker', 10)
        self.pub_vehicle = self.create_publisher(Marker, 'vehicle_marker', 10)
        self.pub_rectangle_frame = self.create_publisher(Marker, 'rectangle_frame_marker', 10)

        # タイマーで軸ラベルを定期的に表示
        self.create_timer(1.0, self.publish_axis_labels)
        # タイマーでカメラの向きの矢印を定期的に表示
        self.create_timer(1.0, self.publish_camera_arrow)
        # タイマーで重機の形を定期的に表示
        self.create_timer(1.0, self.publish_vehicle_marker)
        # タイマーで四角い枠を定期的に表示
        self.create_timer(1.0, self.publish_rectangle_frame)

        # 座標の初期化
        self.point_A = None
        self.point_B = None

    def callback(self, msg_A, msg_B):
        # 木材のマーカー(線分)を表示
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "wood"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        #marker.lifetime = Duration(sec=1)
        marker.scale.x = 0.1  # 線の太さ
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = [msg_A.point, msg_B.point]  # 2点を結ぶ線分
        self.pub_wood.publish(marker)

        # 木材のマーカー(線分)の上に座標を表示
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "wood_point"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        #marker.lifetime = Duration(sec=1)
        cx, cy ,cz = (msg_A.point.x - msg_B.point.x) / 2, (msg_A.point.y - msg_B.point.y) / 2, (msg_A.point.z - msg_B.point.z) / 2  #中心座標を計算
        marker.pose.position.x = cx
        marker.pose.position.y = cy
        marker.pose.position.z = cz + 0.35
        #marker.scale.x = 0.1  # 文字の幅(無効？)
        #marker.scale.y = 0.1  # 文字の高さ(無効？)
        marker.scale.z = 0.4  # 文字のサイズ
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"(x={cx:.2f},y={cy:.2f},z={cz:.2f})"
        self.pub_wood_point.publish(marker)

    # 原点の座標軸にX,Y,Zのラベルを表示
    def publish_axis_labels(self):
        labels = [
            ("X", 1.5, 0.0, 0.0, 0, (1.0, 0.0, 0.0)),
            ("Y", 0.0, 1.5, 0.0, 1, (0.0, 1.0, 0.0)),
            ("Z", 0.0, 0.0, 1.5, 2, (0.0, 0.0, 1.0)),
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
            marker.scale.z = 0.5  # 文字のサイズ
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
        marker.scale.x = 1.3
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 1.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.8
        self.pub_camera_arrow.publish(marker)

    # 重機の形を表示
    def publish_vehicle_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vehicle"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        #marker.lifetime = Duration(sec=1)
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = - 1.05
        marker.scale.x = 2.0
        marker.scale.y = 2.0
        marker.scale.z = 2.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.2
        self.pub_vehicle.publish(marker)

    # 四角い枠を表示
    def publish_rectangle_frame(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rectangle_frame"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # 線の太さ
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        # 頂点の座標を半時計回りに定義
        marker.points = [
            Point(x=-1.1, y=-0.5, z=-2.1),  #左上(重機から見て)
            Point(x=-0.7, y=-0.5, z=-2.1),  #左下
            Point(x=-0.7, y=0.5, z=-2.1),  #右下
            Point(x=-1.1, y=0.5, z=-2.1),  #右上
            Point(x=-1.1, y=-0.5, z=-2.1),  # 始点に戻る
        ]
        self.pub_rectangle_frame.publish(marker)

# 初期化
rclpy.init()

# クラスをインスタンス化
wood_marker_publisher = WoodMarkerPublisher()

# spin()でループ処理を実行
rclpy.spin(wood_marker_publisher)

# 終了
wood_marker_publisher.destroy_node()
rclpy.shutdown()