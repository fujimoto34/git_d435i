# 石の座標をサブスクライブしてMarkerをパブリッシュ+重機を追加
# python3 stone_marker2.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
#from builtin_interfaces.msg import Duration
#from std_msgs.msg import Header
from sensor_msgs.msg import Imu
#import math
#import tf_transformations  # python3 -m pip install tf-transformationsが必要(失敗)
import numpy as np
#import quaternion  # pip install numpy-quaternionが必要(失敗)


class StoneMarkerPublisher(Node):
    def __init__(self):
        super().__init__('stone_marker_publisher')
        self.sub = self.create_subscription(Point, 'stone_xyz', self.callback, 10)
        self.pub_stone = self.create_publisher(Marker, 'stone_marker', 10)
        self.pub_stone_point = self.create_publisher(Marker, 'stone_point_marker', 10)
        self.pub_axis_labels = self.create_publisher(Marker, 'axis_labels_marker', 10)
        self.pub_camera_arrow = self.create_publisher(Marker, 'camera_arrow_marker', 10)
        self.pub_vehicle = self.create_publisher(Marker, 'vehicle_marker', 10)
        self.pub_rectangle_frame = self.create_publisher(Marker, 'rectangle_frame_marker', 10)
        self.sub_arm_angle = self.create_subscription(Imu, '/ssl0/imu/data_arm', self.callback_arm, 10)
        self.sub_att_angle = self.create_subscription(Imu, '/ssl0/imu/data_att', self.callback_att, 10)
        self.pub_machine = self.create_publisher(Marker, 'machine_marker', 10)

        # タイマーで軸ラベルを定期的に表示
        self.create_timer(1.0, self.publish_axis_labels)
        # タイマーでカメラの向きの矢印を定期的に表示
        self.create_timer(1.0, self.publish_camera_arrow)
        # タイマーで重機の形(立方体)を定期的に表示
        self.create_timer(1.0, self.publish_vehicle_marker)
        # タイマーで四角い枠を定期的に表示
        self.create_timer(1.0, self.publish_rectangle_frame)
        # タイマーでスキッドステアローダーの形を定期的に表示
        self.create_timer(0.3, self.publish_machine_marker)

        # アームとアタッチメントの初期角度を定義
        self.current_arm_angle = -1/6 * np.pi
        self.current_att_angle = 0.0

        print('Publishing now.')

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
        marker.pose.position.x = msg.x
        marker.pose.position.y = msg.y
        marker.pose.position.z = msg.z - 0.1
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
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
        marker.pose.position.z = msg.z + 0.35
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
        marker.text = f"(x={msg.x:.2f},y={msg.y:.2f},z={msg.z:.2f})"
        self.pub_stone_point.publish(marker)

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

    # 重機の形(立方体)を表示
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
        marker.scale.y = 1.8
        marker.scale.z = 2.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.3
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

    # アームの角度をサブスクライブして代入
    def callback_arm(self, msg):
        self.current_arm_angle = msg.orientation.x * np.pi/180 - np.pi

    # アタッチメントの角度をサブスクライブして代入
    def callback_att(self, msg):
        self.current_att_angle = msg.orientation.x * np.pi/180 - 2/3*np.pi

    # スキッドステアローダーの形を表示
    def publish_machine_marker(self):
        # 車体を表示
    #     marker = Marker()
    #     marker.header.frame_id = 'map'
    #     marker.header.stamp = self.get_clock().now().to_msg()
    #     marker.ns = "vehicle"
    #     marker.id = 0
    #     marker.type = Marker.CUBE
    #     marker.action = Marker.ADD
    #     marker.scale.x = 1.500
    #     marker.scale.y = 1.675
    #     marker.scale.z = 1.770
    #     marker.pose.position.x = 0.0
    #     marker.pose.position.y = 0.0
    #     marker.pose.position.z = 0.0
    #     marker.pose.orientation.x = 0.0
    #     marker.pose.orientation.y = 0.0
    #     marker.pose.orientation.z = 0.0
    #     marker.pose.orientation.w = 1.0
    #     marker.color.r = 0.0
    #     marker.color.g = 1.0
    #     marker.color.b = 1.0
    #     marker.color.a = 0.5
    #     self.pub_machine.publish(marker)

        # オイラー角(rad)をクォータニオン(x, y, z, w)に変換する関数
        def euler_to_quaternion(roll, pitch, yaw):  # roll, pitch, yawの回転軸はそれぞれx,y,z
            cy = np.cos(yaw * 0.5)
            sy = np.sin(yaw * 0.5)
            cp = np.cos(pitch * 0.5)
            sp = np.sin(pitch * 0.5)
            cr = np.cos(roll * 0.5)
            sr = np.sin(roll * 0.5)
            q = np.zeros(4)
            q[0] = sr * cp * cy - cr * sp * sy  # x
            q[1] = cr * sp * cy + sr * cp * sy  # y
            q[2] = cr * cp * sy - sr * sp * cy  # z
            q[3] = cr * cp * cy + sr * sp * sy  # w
            return q

        # 左アームを表示
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "left_arm"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 3.482
        marker.scale.y = 0.1
        marker.scale.z = 0.2
        x, y, z, w = euler_to_quaternion(0, self.current_arm_angle, 0)  # 回転してから平行移動
        marker.pose.orientation.x = x
        marker.pose.orientation.y = y
        marker.pose.orientation.z = z
        marker.pose.orientation.w = w
        offset_x = 3.482 / 2 - (3.482 / 2 * np.cos(self.current_arm_angle))  # 角度0のとき根本のx座標と角度θのときのずれた根本のx座標の差
        offset_z = 3.482 / 2 * np.sin(self.current_arm_angle)
        marker.pose.position.x = 0.4 + offset_x
        marker.pose.position.y = - 0.95
        marker.pose.position.z = -0.3 + offset_z
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        self.pub_machine.publish(marker)
        # 右アームを表示
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "right_arm"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 3.482
        marker.scale.y = 0.1
        marker.scale.z = 0.2
        x, y, z, w = euler_to_quaternion(0, self.current_arm_angle, 0)  # 回転してから平行移動
        marker.pose.orientation.x = x
        marker.pose.orientation.y = y
        marker.pose.orientation.z = z
        marker.pose.orientation.w = w
        offset_x = 3.482 / 2 - (3.482 / 2 * np.cos(self.current_arm_angle))  # 角度0のとき根本のx座標と角度θのときのずれた根本のx座標の差
        offset_z = 3.482 / 2 * np.sin(self.current_arm_angle)
        marker.pose.position.x = 0.4 + offset_x
        marker.pose.position.y = 0.95
        marker.pose.position.z = -0.3 + offset_z
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        self.pub_machine.publish(marker)
        # アームの角度を表示
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "arm_angle"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.4 + offset_x + (3.482 / 2) * np.cos(self.current_arm_angle)
        marker.pose.position.y = 0.0
        marker.pose.position.z = -0.3 + offset_z - (3.482 / 2) * np.sin(self.current_arm_angle) + 0.4
        marker.scale.z = 0.4  # 文字のサイズ
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"{self.current_arm_angle*180/np.pi:.1f}"
        self.pub_machine.publish(marker)

        # アタッチメントを表示
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "att"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.8
        marker.scale.z = 0.6
        x, y, z, w = euler_to_quaternion(0, self.current_att_angle, 0)  # 回転してから平行移動
        marker.pose.orientation.x = x
        marker.pose.orientation.y = y
        marker.pose.orientation.z = z
        marker.pose.orientation.w = w
        offset_x = - (3.482 / 2 * np.cos(self.current_arm_angle)) + 3.482 / 2 - (3.482 / 2 * np.cos(self.current_arm_angle))
        offset_z = 3.482 / 2 * np.sin(self.current_arm_angle) + 3.482 / 2 * np.sin(self.current_arm_angle)
        marker.pose.position.x = 0.4 + offset_x
        marker.pose.position.y = 0.0
        marker.pose.position.z = - 0.1 + offset_z
        marker.color.r = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        self.pub_machine.publish(marker)
        # アタッチメントの角度を表示
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "att_angle"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.4 + offset_x
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5 + offset_z
        marker.scale.z = 0.4  # 文字のサイズ
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        if -15 < self.current_att_angle*180/np.pi < -5:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
        marker.text = f"{self.current_att_angle*180/np.pi:.1f}"
        self.pub_machine.publish(marker)


# 初期化
rclpy.init()

# クラスをインスタンス化
stone_marker_publisher = StoneMarkerPublisher()

# spin()でループ処理を実行
rclpy.spin(stone_marker_publisher)

# 終了
stone_marker_publisher.destroy_node()
rclpy.shutdown()