#!/usr/bin/env python3
import argparse
import csv
import threading
from collections import deque
from time import sleep

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import matplotlib
matplotlib.use("TkAgg")  # 필요시 다른 백엔드로 바꿔도 됨 (Qt5Agg 등)
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class RxMonitor(Node):
    def __init__(self, topic, target_period):
        super().__init__('encoder_listener_plot')
        self.sub = self.create_subscription(
            Float32MultiArray, topic, self.cb, 10
        )
        # 상태값
        self.prev_rx = None
        self.prev_tx = None
        self.tx0 = None
        self.rx0 = None
        self.target_period = float(target_period)

        # 공유 버퍼 (플롯 스레드에서 읽음)
        self.lock = threading.Lock()
        self.t_rel = deque(maxlen=20000)      # s (rx 기준 상대시간)
        self.dt_rx_ms = deque(maxlen=20000)   # ms
        self.lat_ms = deque(maxlen=20000)     # ms
        self.dt_tx_ms = deque(maxlen=20000)   # ms (참조용)
        self.jitter_ms = deque(maxlen=20000)  # dt_rx - dt_tx (ms)

        # 통계
        self.samples = 0
        self.sum_dt_rx = 0.0
        self.sum_sq_dt_rx = 0.0

        # CSV
        self.csv_writer = None
        self.csv_file = None

    def enable_csv(self, path):
        self.csv_file = open(path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["t_rel_s", "angle_deg", "speed_deg_s",
                                  "tx_s", "rx_s",
                                  "dt_tx_s", "dt_rx_s",
                                  "lat_ms", "jitter_ms"])

    def close_csv(self):
        if self.csv_file:
            self.csv_file.close()

    def now_sec(self):
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9

    def cb(self, msg):
        if len(msg.data) < 3:
            self.get_logger().warn(f"데이터 부족: {list(msg.data)}")
            return
        angle, speed, tx = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
        rx = self.now_sec()

        # 기준점 설정
        if self.tx0 is None:
            self.tx0 = tx
        if self.rx0 is None:
            self.rx0 = rx

        # 주기 계산
        dt_tx = (tx - self.prev_tx) if self.prev_tx is not None else float('nan')
        dt_rx = (rx - self.prev_rx) if self.prev_rx is not None else float('nan')

        # 지연 변화량 (오프셋 제거)
        lat = (rx - self.rx0) - (tx - self.tx0)

        # 상대 시간 (rx 기준)
        t_rel = rx - self.rx0

        # 공유 버퍼 업데이트
        with self.lock:
            self.t_rel.append(t_rel)
            self.dt_rx_ms.append(dt_rx * 1000.0 if dt_rx == dt_rx else float('nan'))
            self.lat_ms.append(lat * 1000.0)
            self.dt_tx_ms.append(dt_tx * 1000.0 if dt_tx == dt_tx else float('nan'))
            self.jitter_ms.append((dt_rx - dt_tx) * 1000.0 if (dt_rx == dt_rx and dt_tx == dt_tx) else float('nan'))

            if dt_rx == dt_rx:  # not NaN
                self.samples += 1
                self.sum_dt_rx += dt_rx
                self.sum_sq_dt_rx += dt_rx * dt_rx

        # CSV 로깅
        if self.csv_writer:
            self.csv_writer.writerow([
                f"{t_rel:.6f}", f"{angle:.6f}", f"{speed:.6f}",
                f"{tx:.6f}", f"{rx:.6f}",
                f"{dt_tx:.6f}" if dt_tx == dt_tx else "",
                f"{dt_rx:.6f}" if dt_rx == dt_rx else "",
                f"{lat*1000.0:.6f}", f"{(dt_rx - dt_tx)*1000.0:.6f}" if (dt_rx == dt_rx and dt_tx == dt_tx) else ""
            ])

        self.prev_tx = tx
        self.prev_rx = rx


def spin_node(node):
    # rclpy.spin()은 블로킹이라 별도 스레드로 돌림
    rclpy.spin(node)


def main():
    parser = argparse.ArgumentParser(description="Plot dt_rx & latency in real time")
    parser.add_argument("--topic", default="encoder_data", help="ROS2 topic name")
    parser.add_argument("--window", type=float, default=10.0, help="seconds of data to display")
    parser.add_argument("--target", type=float, default=0.1, help="target period in seconds (guide line)")
    parser.add_argument("--csv", type=str, default="", help="optional CSV log path")
    args = parser.parse_args()

    rclpy.init()
    node = RxMonitor(topic=args.topic, target_period=args.target)

    if args.csv:
        node.enable_csv(args.csv)

    # ROS 스레드 시작
    th = threading.Thread(target=spin_node, args=(node,), daemon=True)
    th.start()

    # Matplotlib 준비
    fig = plt.figure(figsize=(10, 7))
    gs = fig.add_gridspec(2, 1, height_ratios=[2, 1], hspace=0.25)
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[1, 0], sharex=ax1)

    # dt_rx 라인
    (line_dt_rx,) = ax1.plot([], [], lw=1.5, label="dt_rx (ms)")
    # 가이드라인(목표 주기)
    (line_target,) = ax1.plot([], [], linestyle="--", lw=1.0, label=f"target {args.target*1000:.1f} ms")

    ax1.set_ylabel("dt_rx (ms)")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    # lat 라인
    (line_lat,) = ax2.plot([], [], lw=1.2, label="lat (ms)")
    ax2.set_xlabel("time since first rx (s)")
    ax2.set_ylabel("lat (ms)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right")

    # 주기적 업데이트 함수
    def update(_):
        with node.lock:
            if len(node.t_rel) == 0:
                return line_dt_rx, line_target, line_lat

            t = list(node.t_rel)
            dt_rx_ms = list(node.dt_rx_ms)
            lat_ms = list(node.lat_ms)

        # 윈도우 자르기
        tmax = t[-1]
        tmin = max(1.0, tmax - args.window)
        # 인덱스 선정
        i0 = 0
        for i in range(len(t)):
            if t[i] >= tmin:
                i0 = i
                break
        t_win = t[i0:]
        dt_rx_win = dt_rx_ms[i0:]
        lat_win = lat_ms[i0:]

        # 데이터 설정
        line_dt_rx.set_data(t_win, dt_rx_win)
        line_lat.set_data(t_win, lat_win)

        # 목표 주기 가이드라인
        line_target.set_data([tmin, tmax], [args.target * 1000.0, args.target * 1000.0])

        # 축 범위
        if len(t_win) >= 2:
            ax1.set_xlim(tmin, tmax)
            # y범위 자동 스케일 + 여유
            if any(x == x for x in dt_rx_win):  # not all NaN
                vals = [x for x in dt_rx_win if x == x]
                ymin = min(vals + [args.target * 1000.0])
                ymax = max(vals + [args.target * 1000.0])
                pad = max(0.5, 0.1 * (ymax - ymin) if ymax > ymin else 1.0)
                ax1.set_ylim(ymin - pad, ymax + pad)

            if any(x == x for x in lat_win):
                ymin2 = min(lat_win)
                ymax2 = max(lat_win)
                pad2 = max(0.5, 0.1 * (ymax2 - ymin2) if ymax2 > ymin2 else 1.0)
                ax2.set_ylim(ymin2 - pad2, ymax2 + pad2)

        # 제목에 간단 통계 표시
        with node.lock:
            n = node.samples
            if n >= 2:
                mean = node.sum_dt_rx / n
                var = node.sum_sq_dt_rx / n - mean * mean
                std = (var ** 0.5) if var > 0 else 0.0
                fig.suptitle(f"dt_rx mean={mean*1000:.3f} ms, std={std*1000:.3f} ms, n={n}")
            else:
                fig.suptitle("Waiting for data...")

        return line_dt_rx, line_target, line_lat

    ani = FuncAnimation(fig, update, interval=100)  # 10 Hz로 화면 갱신
    try:
        plt.show()
    finally:
        node.close_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

