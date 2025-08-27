#!/usr/bin/env python3
import argparse
import csv
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import matplotlib
matplotlib.use("TkAgg")  # 필요시 다른 백엔드로 변경 가능 (예: Qt5Agg)
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class AngleMonitor(Node):
    def __init__(self, topic: str, window: float):
        super().__init__('angle_listener_plot')
        self.sub = self.create_subscription(
            Float32MultiArray, topic, self.cb, 10
        )

        # 상태값
        self.rx0 = None  # 첫 수신 시간 (상대시간 기준점)

        # 공유 버퍼 (플롯 스레드에서 읽음)
        self.lock = threading.Lock()
        self.t_rel = deque(maxlen=20000)   # s (rx 기준 상대시간)
        self.angle = deque(maxlen=20000)   # deg
        self.window = float(window)

        # 통계 (화면 표시에는 사용하지 않음)
        self.samples = 0
        self.sum_angle = 0.0
        self.sum_sq_angle = 0.0

        # CSV
        self.csv_writer = None
        self.csv_file = None

    def enable_csv(self, path: str):
        self.csv_file = open(path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["t_rel_s", "angle_deg"])

    def close_csv(self):
        if self.csv_file:
            self.csv_file.close()

    def now_sec(self) -> float:
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9

    def cb(self, msg: Float32MultiArray):
        if len(msg.data) < 1:
            self.get_logger().warn(f"데이터 부족: {list(msg.data)}")
            return

        angle = float(msg.data[0])  # 각도만 사용
        rx = self.now_sec()

        # 기준점 설정 (첫 수신 시각)
        if self.rx0 is None:
            self.rx0 = rx

        # 상대 시간
        t_rel = rx - self.rx0

        # 공유 버퍼 업데이트
        with self.lock:
            self.t_rel.append(t_rel)
            self.angle.append(angle)

            # 통계 (평균/표준편차 계산만, 화면 표시 X)
            self.samples += 1
            self.sum_angle += angle
            self.sum_sq_angle += angle * angle

        # CSV 로깅
        if self.csv_writer:
            self.csv_writer.writerow([f"{t_rel:.6f}", f"{angle:.6f}"])


def spin_node(node: Node):
    # rclpy.spin()은 블로킹이라 별도 스레드로 돌림
    rclpy.spin(node)


def main():
    parser = argparse.ArgumentParser(description="Plot angle (deg) in real time")
    parser.add_argument("--topic", default="encoder_data", help="ROS2 topic name")
    parser.add_argument("--window", type=float, default=10.0, help="seconds of data to display")
    parser.add_argument("--csv", type=str, default="", help="optional CSV log path")
    args = parser.parse_args()

    rclpy.init()
    node = AngleMonitor(topic=args.topic, window=args.window)

    if args.csv:
        node.enable_csv(args.csv)

    # ROS 스레드 시작
    th = threading.Thread(target=spin_node, args=(node,), daemon=True)
    th.start()

    # Matplotlib 준비 (각도만 하나의 축에 표시)
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(1, 1, 1)

    (line_angle,) = ax.plot([], [], lw=1.5, label="angle (deg)")
    ax.set_xlabel("time since first rx (s)")
    ax.set_ylabel("angle (deg)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    # 주기적 업데이트 함수
    def update(_):
        with node.lock:
            if len(node.t_rel) == 0:
                fig.suptitle("Waiting for data...")
                return line_angle

            t = list(node.t_rel)
            ang = list(node.angle)

        # 윈도우 자르기
        tmax = t[-1]
        tmin = max(0.0, tmax - node.window)

        # 인덱스 선정
        i0 = 0
        for i in range(len(t)):
            if t[i] >= tmin:
                i0 = i
                break
        t_win = t[i0:]
        ang_win = ang[i0:]

        # 데이터 설정
        line_angle.set_data(t_win, ang_win)

        # 축 범위
        if len(t_win) >= 2:
            ax.set_xlim(tmin, tmax)
            # y범위 자동 스케일 + 여유
            ymin = min(ang_win)
            ymax = max(ang_win)
            pad = max(0.5, 0.1 * (ymax - ymin) if ymax > ymin else 1.0)
            ax.set_ylim(ymin - pad, ymax + pad)

        # 제목: 현재 각도만 표시
        current_angle = ang_win[-1]
        fig.suptitle(f"angle = {current_angle:.3f} deg")

        return line_angle

    ani = FuncAnimation(fig, update, interval=50)  # 약 20 Hz로 화면 갱신
    try:
        plt.show()
    finally:
        node.close_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

