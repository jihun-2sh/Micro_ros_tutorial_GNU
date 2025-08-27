#!/usr/bin/env python3
import argparse
import csv
import sys
import signal
from time import sleep

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# 런타임 중엔 matplotlib를 불러오지 않음(오버헤드 최소화).
# 종료 후 플롯할 때만 import.


class RxMonitor(Node):
    def __init__(self, topic, target_period):
        super().__init__('encoder_listener_logger')
        self.sub = self.create_subscription(
            Float32MultiArray, topic, self.cb, 10
        )

        # 상태값
        self.prev_rx = None
        self.prev_tx = None
        self.tx0 = None
        self.rx0 = None
        self.target_period = float(target_period)

        # 통계 (dt_rx)
        self.samples = 0
        self.sum_dt_rx = 0.0
        self.sum_sq_dt_rx = 0.0

        # CSV
        self.csv_writer = None
        self.csv_file = None
        self.csv_path = None

    def enable_csv(self, path):
        self.csv_path = path
        self.csv_file = open(path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["t_rel_s", "angle_deg", "speed_deg_s",
                                  "tx_s", "rx_s",
                                  "dt_tx_s", "dt_rx_s",
                                  "lat_ms", "jitter_ms"])
        # 즉시 flush해서 데이터 유실 방지
        self.csv_file.flush()

    def close_csv(self):
        if self.csv_file:
            try:
                self.csv_file.flush()
            except Exception:
                pass
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

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

        # 통계 업데이트
        if dt_rx == dt_rx:  # not NaN
            self.samples += 1
            self.sum_dt_rx += dt_rx
            self.sum_sq_dt_rx += dt_rx * dt_rx

        # CSV 로깅 (런타임엔 오직 기록만!)
        if self.csv_writer:
            self.csv_writer.writerow([
                f"{t_rel:.6f}", f"{angle:.6f}", f"{speed:.6f}",
                f"{tx:.6f}", f"{rx:.6f}",
                f"{dt_tx:.6f}" if dt_tx == dt_tx else "",
                f"{dt_rx:.6f}" if dt_rx == dt_rx else "",
                f"{lat*1000.0:.6f}",
                f"{(dt_rx - dt_tx)*1000.0:.6f}" if (dt_rx == dt_rx and dt_tx == dt_tx) else ""
            ])
            # 디스크로 바로 밀어내서 안전하게
            self.csv_file.flush()

        self.prev_tx = tx
        self.prev_rx = rx


def plot_after_exit(csv_path, window=None):
    """프로그램 종료 후 CSV에서 dt_rx(ms) vs t_rel(s) 정적 플롯."""
    if not csv_path:
        print("[INFO] CSV 경로가 없어 플롯을 생략합니다.", file=sys.stderr)
        return

    # 종료 후에만 matplotlib import (오버헤드 최소화)
    import matplotlib
    matplotlib.use("TkAgg")  # 필요 시 다른 백엔드 사용 가능
    import matplotlib.pyplot as plt

    t_rel = []
    dt_rx_ms = []

    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = float(row["t_rel_s"])
                dt_rx_s_str = row.get("dt_rx_s", "").strip()
                if dt_rx_s_str == "":
                    continue
                dt = float(dt_rx_s_str) * 1000.0  # ms
                t_rel.append(t)
                dt_rx_ms.append(dt)
            except Exception:
                continue

    if not t_rel:
        print("[WARN] CSV에 유효한 데이터(dt_rx)가 없어 플롯을 건너뜁니다.", file=sys.stderr)
        return

    # 윈도우 지정 시 마지막 window초 구간만 그림
    if window is not None and len(t_rel) > 1:
        tmax = t_rel[-1]
        tmin = max(0.0, tmax - window)
        start_idx = 0
        for i, t in enumerate(t_rel):
            if t >= tmin:
                start_idx = i
                break
        t_rel = t_rel[start_idx:]
        dt_rx_ms = dt_rx_ms[start_idx:]

    # 통계 (플롯 타이틀용)
    import math
    n = len(dt_rx_ms)
    mean = sum(dt_rx_ms) / n
    var = sum((x - mean) ** 2 for x in dt_rx_ms) / n if n > 1 else 0.0
    std = math.sqrt(var)

    plt.figure(figsize=(10, 5))
    plt.plot(t_rel, dt_rx_ms, lw=1.2, label="dt_rx (ms)")
    plt.grid(True, alpha=0.3)
    plt.xlabel("time since first rx (s)")
    plt.ylabel("dt_rx (ms)")
    plt.title(f"dt_rx vs time  |  mean={mean:.3f} ms, std={std:.3f} ms, n={n}")
    plt.legend(loc="upper right")
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Log encoder timing to CSV; plot after exit.")
    parser.add_argument("--topic", default="encoder_data", help="ROS2 topic name")
    parser.add_argument("--target", type=float, default=0.1, help="target period in seconds (for stats only)")
    parser.add_argument("--csv", type=str, required=True, help="CSV log path (required)")
    parser.add_argument("--plot-window", type=float, default=None,
                        help="종료 후 플롯 시, 마지막 N초만 표시 (예: 10.0). 미지정 시 전체.")
    args = parser.parse_args()

    rclpy.init()
    node = RxMonitor(topic=args.topic, target_period=args.target)
    node.enable_csv(args.csv)

    # Ctrl+C 처리 깔끔하게
    def handle_sigint(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 통계 출력
        if node.samples >= 2:
            mean = node.sum_dt_rx / node.samples
            var = node.sum_sq_dt_rx / node.samples - mean * mean
            std = (var ** 0.5) if var > 0 else 0.0
            print(f"[STATS] dt_rx mean={mean*1000:.3f} ms, std={std*1000:.3f} ms, n={node.samples}")
        else:
            print("[STATS] 샘플이 부족합니다.")

        # 자원 정리
        node.close_csv()
        node.destroy_node()
        rclpy.shutdown()

        # 종료 후 플롯
        try:
            plot_after_exit(args.csv, window=args.plot_window)
        except Exception as e:
            print(f"[WARN] 종료 후 플롯 중 오류: {e}", file=sys.stderr)


if __name__ == "__main__":
    main()

