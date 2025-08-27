#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class EncoderListener(Node):
    def __init__(self):
        super().__init__('encoder_listener')
        self.subscription = self.create_subscription(
            Float32MultiArray, 'encoder_data', self.listener_callback, 10
        )
        # 상태값
        self.prev_rx = None
        self.prev_tx = None
        self.tx0 = None
        self.rx0 = None

    def now_sec(self):
        # ROS clock -> float seconds
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9

    def listener_callback(self, msg):
        if len(msg.data) < 3:
            print(f"데이터 부족: {msg.data}")
            return

        angle = float(msg.data[0])
        speed = float(msg.data[1])
        tx    = float(msg.data[2])  # Teensy가 보낸 시각(초)
        rx    = self.now_sec()      # PC에서 받은 시각(초)

        # 기준 시각(오프셋 제거용)
        if self.tx0 is None:
            self.tx0 = tx
        if self.rx0 is None:
            self.rx0 = rx

        dt_tx = (tx - self.prev_tx) if self.prev_tx is not None else float('nan')
        dt_rx = (rx - self.prev_rx) if self.prev_rx is not None else float('nan')

        # 지연 변화 추정: (수신누적) - (송신누적)
        latency_est = (rx - self.rx0) - (tx - self.tx0)

        print(
            f"각도: {angle:.2f} deg\t속도: {speed:.2f} deg/s\t"
            f"tx: {tx:.6f}s\trx: {rx:.6f}s\t"
            f"dt_tx: {dt_tx:.4f}s\tdt_rx: {dt_rx:.4f}s\t"
            f"lat≈ {latency_est*1000:.2f} ms"
        )

        self.prev_tx = tx
        self.prev_rx = rx

def main(args=None):
    rclpy.init(args=args)
    node = EncoderListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

