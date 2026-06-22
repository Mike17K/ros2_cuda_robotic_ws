#!/usr/bin/env python3
"""
Automated tuning sweep for `drone_controller`.

What it does:
- Iterates a grid of controller parameters (rate PID + attitude gain + torque gain)
- For each set: uses `ros2 param set /drone_controller ...` to apply params
- Sends a short joystick step (publishes to `/joy`) to command a small attitude
- Subscribes to `/drone/imu` and records roll/pitch response
- Computes simple metrics (peak, settling time, RMS) and ranks parameter sets
- Emits a recommended `ros2 param set` sequence for the best-performing set

Requirements:
- Source your ROS 2 workspace (so `ros2` is on PATH) and start the simulator + controller
- Python deps: rclpy (ROS 2), numpy

Usage:
  ./tune_sweep.py --duration 4 --target_deg 6

Note: run with the simulator and `drone_controller` already launched.
"""

import argparse
import csv
import math
import os
import subprocess
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy

import numpy as np


def quat_to_euler(x, y, z, w):
    # return roll, pitch, yaw
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class IMURecorder(Node):
    def __init__(self, topic='/drone/imu'):
        super().__init__('tune_imu_recorder')
        self.sub = self.create_subscription(Imu, topic, self.cb, 10)
        self.lock = threading.Lock()
        self.data = []  # list of (t, roll, pitch)
        self.start_time = None

    def cb(self, msg: Imu):
        with self.lock:
            t = time.time()
            if self.start_time is None:
                self.start_time = t
            r, p, y = quat_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            self.data.append((t - self.start_time, r, p))

    def reset(self):
        with self.lock:
            self.data = []
            self.start_time = None

    def get_data(self):
        with self.lock:
            return list(self.data)


def ros2_set_param(param, value, node_name='/drone_controller'):
    # call `ros2 param set` for simplicity and robustness
    cmd = ['ros2', 'param', 'set', node_name, param, str(value)]
    try:
        subprocess.check_output(cmd, stderr=subprocess.STDOUT)
        return True
    except subprocess.CalledProcessError as e:
        print('Failed to set param:', cmd, e.output.decode())
        return False


def publish_joy_step(pub, target_axes, hold_time=0.5, rate=50):
    # publish target_axes (list) as a single step for hold_time seconds
    msg = Joy()
    msg.axes = [0.0] * max(4, len(target_axes))
    for i, v in enumerate(target_axes):
        msg.axes[i] = float(v)
    msg.buttons = []

    # keep sending for hold_time
    interval = 1.0 / rate
    end = time.time() + hold_time
    while time.time() < end:
        pub.publish(msg)
        time.sleep(interval)

    # then send zeros
    msg.axes = [0.0] * len(msg.axes)
    pub.publish(msg)


def compute_metrics(data, target_rad):
    # data: list of (t, roll, pitch)
    if len(data) == 0:
        return {'peak': 1e6, 'settling': 1e6, 'rms': 1e6}
    times = np.array([d[0] for d in data])
    roll = np.array([d[1] for d in data])
    pitch = np.array([d[2] for d in data])
    # use the axis with larger response
    resp = np.abs(roll) if np.max(np.abs(roll)) >= np.max(np.abs(pitch)) else np.abs(pitch)
    peak = float(np.max(resp))
    # RMS error to target
    err = resp - abs(target_rad)
    rms = float(np.sqrt(np.mean(err**2)))
    # settling: time after which response stays within 5% of target
    tol = 0.05 * abs(target_rad)
    settling = float(1e6)
    if peak == 0:
        settling = 1e6
    else:
        # find first time after step (t>0.1s) where all future samples within tol
        for i, t in enumerate(times):
            if t < 0.05:
                continue
            window = resp[i:]
            if np.all(np.abs(window - abs(target_rad)) <= tol):
                settling = float(times[i])
                break
    return {'peak': peak, 'settling': settling, 'rms': rms}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=4.0, help='record duration per trial (s)')
    parser.add_argument('--target_deg', type=float, default=5.0, help='step target angle in degrees')
    parser.add_argument('--node', type=str, default='/drone_controller', help='controller node name')
    parser.add_argument('--joy_topic', type=str, default='/joy', help='joy topic to publish step')
    parser.add_argument('--imu_topic', type=str, default='/drone/imu', help='imu topic to record')
    parser.add_argument('--out', type=str, default='tune_results.csv', help='results CSV')
    args = parser.parse_args()

    # parameter grid (small defaults — tune ranges as needed)
    kp_list = [4.0, 6.0, 8.0]
    ki_list = [0.0, 0.05, 0.1]
    kd_list = [0.0, 0.002]
    Kr_list = [6.0, 8.0]
    torque_list = [0.5, 0.7]

    rclpy.init()
    node = Node('tune_driver')
    imu_rec = IMURecorder(topic=args.imu_topic)
    joy_pub = node.create_publisher(Joy, args.joy_topic, 10)

    target_rad = math.radians(args.target_deg)

    results = []

    total = len(kp_list) * len(ki_list) * len(kd_list) * len(Kr_list) * len(torque_list)
    trial_idx = 0

    for kp in kp_list:
        for ki in ki_list:
            for kd in kd_list:
                for Kr in Kr_list:
                    for torque in torque_list:
                        trial_idx += 1
                        print(f'Trial {trial_idx}/{total}: kp={kp} ki={ki} kd={kd} Kr={Kr} torque={torque}')
                        # set params via ros2
                        ros2_set_param('rate_Kp', kp, node_name=args.node)
                        ros2_set_param('rate_Ki', ki, node_name=args.node)
                        ros2_set_param('rate_Kd', kd, node_name=args.node)
                        ros2_set_param('K_R_attitude', Kr, node_name=args.node)
                        ros2_set_param('torque_to_speed_gain', torque, node_name=args.node)

                        # reset recorder and wait a bit
                        imu_rec.reset()
                        time.sleep(0.6)

                        # publish step: set roll axis to target (axis 0)
                        # Joy axes: [roll, pitch, yaw, throttle]
                        step_axes = [args.target_deg / args.target_deg, 0.0, 0.0, 0.0]
                        # Actually use normalized axis value: target / max_attitude_deg -> but controller maps axes directly
                        norm = args.target_deg / (args.target_deg if args.target_deg!=0 else 1.0)
                        step_axes = [norm, 0.0, 0.0, 0.0]

                        # publish in separate thread so we can record concurrently
                        tpub = threading.Thread(target=publish_joy_step, args=(joy_pub, step_axes, 0.4))
                        tpub.start()

                        # record for duration
                        t0 = time.time()
                        while time.time() - t0 < args.duration:
                            rclpy.spin_once(node, timeout_sec=0.1)
                            rclpy.spin_once(imu_rec, timeout_sec=0.1)

                        tpub.join()
                        data = imu_rec.get_data()

                        metrics = compute_metrics(data, target_rad)
                        print(' metrics:', metrics)
                        row = {
                            'kp': kp, 'ki': ki, 'kd': kd, 'Kr': Kr, 'torque': torque,
                            'peak': metrics['peak'], 'settling': metrics['settling'], 'rms': metrics['rms']
                        }
                        results.append(row)

                        # short wait between trials
                        time.sleep(0.4)

    # rank by combination of settling + rms (lower better)
    def score(r):
        return r['settling'] + 2.0 * r['rms'] + 0.5 * r['peak']

    results_sorted = sorted(results, key=score)

    # save CSV
    keys = ['kp', 'ki', 'kd', 'Kr', 'torque', 'peak', 'settling', 'rms']
    with open(args.out, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for r in results_sorted:
            writer.writerow({k: r[k] for k in keys})

    best = results_sorted[0]
    print('\nBest result:', best)
    print('\nRecommended ros2 param set commands:')
    print(f"ros2 param set {args.node} rate_Kp {best['kp']}")
    print(f"ros2 param set {args.node} rate_Ki {best['ki']}")
    print(f"ros2 param set {args.node} rate_Kd {best['kd']}")
    print(f"ros2 param set {args.node} K_R_attitude {best['Kr']}")
    print(f"ros2 param set {args.node} torque_to_speed_gain {best['torque']}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
