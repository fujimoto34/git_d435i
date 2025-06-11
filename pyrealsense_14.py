# 検知した木材までの距離と3次元座標(グローバル座標系)をパブリッシュする(空間フィルター+時間フィルター)
# arctan(ゼロ除算エラー)と、show=True(検出速度)を改善。角度固定。
# python3 pyrealsense_14.py

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
#from std_msgs.msg import String
from std_msgs.msg import Float64
#from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped  # 2点同時パブリッシュに使うPoint型

import time


model = YOLO('runs/obb/train/weights/best.pt')
#print(model.names)

# カメラの設定
config = rs.config()
# RGB、深度カメラ
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)
# 加速度センサの有効化
config.enable_stream(rs.stream.accel)

# 深度画像をRGBカメラの視点に位置合わせをするalignモジュールを定義
align = rs.align(rs.stream.color)

# ROS2でPublisherを作成
rclpy.init()
node = Node("wood1")
pub_d = node.create_publisher(Float64, "wood_dis", 10)
pub_xyz_A = node.create_publisher(PointStamped, "wood_xyz_A", 10)
pub_xyz_B = node.create_publisher(PointStamped, "wood_xyz_B", 10)
msg_d = Float64()
msg_xyz_A = PointStamped()
msg_xyz_B = PointStamped()  # AとBで区別したほうが安全らしい

# stream開始
pipeline = rs.pipeline()
profile = pipeline.start(config)

# RGBカメラの内部パラメータを取得(3次元座標取得に必要)
color_intr = rs.video_stream_profile(profile.get_stream(rs.stream.color)).get_intrinsics()

# フィルター準備
# decimation_filterのパラメータ(深度画像の解像度を下げて複雑さを低減(代表値を選ぶor平均を取るなど))
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 1)
# spatial_filterのパラメータ(空間的な平滑化)
spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_magnitude, 1)
spatial.set_option(rs.option.filter_smooth_alpha, 0.25)
spatial.set_option(rs.option.filter_smooth_delta, 50)
# hole_filling_filterのパラメータ(距離の欠損を補完)
hole_filling = rs.hole_filling_filter()
# disparity
depth_to_disparity = rs.disparity_transform(True)
disparity_to_depth = rs.disparity_transform(False)
# 時間的な平滑化フィルターの準備
temporal = rs.temporal_filter()
temporal.set_option(rs.option.filter_smooth_alpha, 0.4)  # 1〜0で、1に近いほど今の値を優先、0に近いほど前の値を重視。デフォルトは0.4
temporal.set_option(rs.option.filter_smooth_delta, 100)  # 何mm以下の差に対してフィルターを施すか。1〜100で、デフォルトは20。
#temporal.set_option(rs.option.persistency_mode, 3)  # 深度データが欠損した場合に時間的な補完をする

try:
    while True:
        # time.sleep(3)
        frames = pipeline.wait_for_frames()

        # 定義した位置合わせを実行
        aligned_frames = align.process(frames)
        
        # frameデータを取得
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame or not color_frame:
            continue

        # フィルターをかける
        filter_frame = decimate.process(depth_frame)
        filter_frame = depth_to_disparity.process(filter_frame)
        filter_frame = spatial.process(filter_frame)
        filter_frame = disparity_to_depth.process(filter_frame)
        filter_frame = hole_filling.process(filter_frame)
        # 時間的な平滑化フィルターをかける
        filter_frame = temporal.process(filter_frame)
        # キャスト(型の変換)
        depth_frame = filter_frame.as_depth_frame()

        # 画像データに変換
        color_image = np.asanyarray(color_frame.get_data())

        # 距離情報をカラースケール画像に変換する
        depth_color_frame = rs.colorizer().colorize(depth_frame)
        # 画像データに変換
        depth_image = np.asanyarray(depth_color_frame.get_data())

        # 深度画像をリアルタイム表示
        # cv2.imshow("Depth Image", depth_image)
        # cv2.waitKey(1)  # なんか必要らしい

        # 加速度データ取得
        accel_frame = frames.first_or_default(rs.stream.accel)
        accel_data = accel_frame.as_motion_frame().get_motion_data()
        # 加速度データ表示
        ax, ay, az = accel_data.x, accel_data.y, accel_data.z
        #print(f"加速度センサ値: x={ax:.3f}, y={ay:.3f}, z={az:.3f}")  # RealSenseの加速度の座標は、左がx正、上がy正、後ろがz正
        # RealSenseの加速度座標をNED座標系の加速度に変換
        ax_ = -az
        ay_ = -ax
        az_ = -ay
        # NED座標系での傾き(ピッチ・ロール)の計算
        pitch_rad = np.arctan2(-ax_, np.sqrt((ay_)**2 + az_**2))
        roll_rad = np.arctan2(ay_, az_)
        pitch = pitch_rad * 180 / np.pi
        roll = roll_rad * 180 / np.pi
        # 傾き表示(NED座標系)
        print(f"カメラのピッチ角（前後の傾き）: {pitch:.2f} 度")  # 後ろが正(y軸周りにzからxに向かう方向が正)
        print(f"カメラのロール角（左右の傾き）: {roll:.2f} 度")  # 右が正(x軸周りにyからzに向かう方向が正)
        pitch_rad = -55.0 * np.pi / 180  # 前に55度で固定
        roll_rad = 0.0 * np.pi /180  # 左右0度で固定
        # x軸回り(ロール)の回転行列(右手系のオブジェクト回転のもの(観測系の回転ではなく))
        R_roll = np.array([
            [1, 0, 0],
            [0, np.cos(roll_rad), -np.sin(roll_rad)],
            [0, np.sin(roll_rad),  np.cos(roll_rad)],
        ])
        # y軸回り(ピッチ)の回転行列(右手系のオブジェクト回転のもの(観測系の回転ではなく))
        R_pitch = np.array([
            [ np.cos(pitch_rad), 0, np.sin(pitch_rad)],
            [0, 1, 0],
            [-np.sin(pitch_rad), 0, np.cos(pitch_rad)],
        ])
        #合成回転行列
        R_total = R_roll @ R_pitch  # どちらかが正しい
        #R_total = R_pitch @ R_roll  # どちらかが正しい

        results = model.predict(color_image, show=False, conf=0.1, imgsz=320, vid_stride=1, stream=False, half=False)  # show=Falseにしたほうが速い

        #print(results[0].names)

        for result in results:
             for obb in result.obb:
                 if int(obb.cls) == 0:  # class 0 is wood
                    x1, y1, x2, y2 = map(int, obb.xyxy[0].tolist())  # 座標を整数に変換。x1,y1は左上、x2,y2は左下
                    # 画像データに検出ボックスを書き込む
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (255, 0, 0), 3)  # 検出ボックス描画
                    label = f"{result.names[int(obb.cls)]} {obb.conf[0]:.2f}"  # テキストを定義
                    (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)  # テキストサイズを取得
                    cv2.rectangle(color_image, (x1, y1 - text_height - 5), (x1 + text_width, y1), (255, 0, 0), -1)  # 文字の背景の四角(青の塗りつぶし)
                    cv2.putText(color_image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)  # テキスト
                    # 木材の両端までの距離を取得
                    cx, cy, w, h, theta = obb.xywhr[0].tolist()  # 中心座標・幅・高さ・角度を取得
                    print(f'obb_theta={theta*180/np.pi:.1f} degrees')
                    if w <= h:  # wを常に長辺にする
                        w, h = h, w
                        theta = theta + np.pi/2
                    print(f'obb_theta={theta*180/np.pi:.1f} degrees')  # thetaを表示
                    dx = (w / 2) * np.cos(theta)
                    dy = (w / 2) * np.sin(theta)
                    xA = cx + dx
                    yA = cy + dy
                    xB = cx - dx
                    yB = cy - dy
                    height, width = color_image.shape[0], color_image.shape[1]  # カメラ画像の縦幅、横幅を取得
                    xA = int(min(max(xA, 0), width -1))  # 両端がカメラの枠外になったときの距離取得エラーを防ぐ
                    yA = int(min(max(yA, 0), height -1))
                    xB = int(min(max(xB, 0), width -1))
                    yB = int(min(max(yB, 0), height -1))
                    #cx = int(min(max(cx, 0), width - 1))  # 中心がカメラの枠外になったときの距離取得エラーを防ぐ
                    #cy = int(min(max(cy, 0), height - 1))  # 中心がカメラの枠外になったときの距離取得エラーを防ぐ
                    cv2.circle(color_image, (xA, yA), 5, (0, 0, 255), -1)  # 両端にマークを描画
                    cv2.circle(color_image, (xB, yB), 5, (0, 0, 255), -1)  # 両端にマークを描画
                    distanceA = depth_frame.get_distance(xA, yA)
                    distanceB = depth_frame.get_distance(xB, yB)
                    if distanceA > 0 and distanceB > 0:
                        #print(f'距離 = {distance:.3f}[m]')
                        #msg_d.data = distance
                        #pub_d.publish(msg_d)
                        # 3次元座標を取得
                        pointA = rs.rs2_deproject_pixel_to_point(color_intr , [xA,yA], distanceA)
                        pointB = rs.rs2_deproject_pixel_to_point(color_intr , [xB,yB], distanceB)
                        #print(f'座標: (x={point[0]:.3f}, y={point[1]:.3f}, z={point[2]:.3f})')  # RealSenseの点の座標は、右がx正、下がy正、前がz正
                        #msg_xyz.x = point[0]
                        #msg_xyz.y = point[1]
                        #msg_xyz.z = point[2]
                        #pub_xyz.publish(msg_xyz)
                        # 座標変換
                        def transform_point(point):
                            # RealSenseの座標点をNED座標系の座標点に変換
                            x = point[2]
                            y = point[0]
                            z = point[1]
                            # カメラ視点の傾いた座標をグローバル座標(NED座標)に変換(前がx,右がy,下がz)
                            cam_point = np.array([x, y, z])
                            world_point = R_total @ cam_point
                            # もとの座標(以前使ってた座標系)に戻す
                            x_ = - world_point[0]
                            y_ =   world_point[1]
                            z_ = - world_point[2]
                            return x_, y_, z_
                        # 変換
                        x_A, y_A, z_A = transform_point(pointA)
                        x_B, y_B, z_B = transform_point(pointB)
                        #print(f'グローバル座標:\nx= {world_point[0]:.3f},\ny= {world_point[1]:.3f},\nz= {world_point[2]:.3f}')  # 前がx、右がy、下がz (RViz上ではxとzの符号が変わり、後ろがx、右がy、上がz(上で変換してるから))
                        # 時間を揃えてパブリッシュ
                        now_time = node.get_clock().now().to_msg()  #1回だけ現在時刻を取得。Clock().now.to_msg()よりも推奨される書き方(importも不要)
                        msg_xyz_A.header.stamp = now_time  # 時間合わせ
                        msg_xyz_A.header.frame_id = 'map'  # フレームid合わせ
                        msg_xyz_A.point.x = x_A  # PointStamped型の座標代入は.pointが必要
                        msg_xyz_A.point.y = y_A
                        msg_xyz_A.point.z = z_A
                        pub_xyz_A.publish(msg_xyz_A)
                        msg_xyz_B.header.stamp = now_time  # 時間合わせ
                        msg_xyz_B.header.frame_id = 'map'  # フレームid合わせ
                        msg_xyz_B.point.x = x_B
                        msg_xyz_B.point.y = y_B
                        msg_xyz_B.point.z = z_B
                        pub_xyz_B.publish(msg_xyz_B)
                    else:
                        print('距離が測定できませんでした')
        
        #print(results[0].names)
        cv2.imshow("Camera View", color_image)  # 検出画面の表示
        cv2.waitKey(1)  # 1ms待機。なんか必要らしい


finally:
    pipeline.stop()
    cv2.destroyAllWindows()