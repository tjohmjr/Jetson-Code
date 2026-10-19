#!/usr/bin/env python3
"""
ROS2 lidar logger: subscribes to /scan (sensor_msgs/LaserScan from sllidar_ros2)
and stores each scan to Parquet (batched) with a session row in SQLite.

Layout on disk:
  <output_dir>/sme.db                                   SQLite (sessions table)
  <output_dir>/lidar/session=<id>/part-00000.parquet    one file per flush

Design notes:
  - One row per scan; ranges/intensities are list<float32> columns.
  - Files are written to a .tmp name and renamed, so a power cut never leaves
    a half-written .parquet file (only the last <flush_seconds> of data is lost).
  - Three timestamps per scan: monotonic (ordering/latency), wall (Chrony-synced),
    and the driver's header stamp.
"""
import os
import sqlite3
import time
import uuid
from datetime import datetime, timezone

import pyarrow as pa
import pyarrow.parquet as pq
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

LIDAR_SCHEMA = pa.schema([
    ("session_id", pa.string()),
    ("scan_idx", pa.int64()),
    ("t_mono_ns", pa.int64()),
    ("t_wall_ns", pa.int64()),
    ("t_msg_ns", pa.int64()),
    ("angle_min", pa.float32()),
    ("angle_max", pa.float32()),
    ("angle_increment", pa.float32()),
    ("range_min", pa.float32()),
    ("range_max", pa.float32()),
    ("scan_time", pa.float32()),
    ("ranges", pa.list_(pa.float32())),
    ("intensities", pa.list_(pa.float32())),
])


class LidarLogger(Node):
    def __init__(self):
        super().__init__("lidar_logger")
        self.declare_parameter("topic", "/scan")
        self.declare_parameter("output_dir", os.path.expanduser("~/sme_data"))
        self.declare_parameter("flush_seconds", 5.0)
        self.declare_parameter("notes", "")

        topic = self.get_parameter("topic").value
        self.output_dir = os.path.expanduser(self.get_parameter("output_dir").value)
        flush_s = float(self.get_parameter("flush_seconds").value)
        notes = self.get_parameter("notes").value

        self.session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:6]
        self.lidar_dir = os.path.join(self.output_dir, "lidar", f"session={self.session_id}")
        os.makedirs(self.lidar_dir, exist_ok=True)

        self._init_db(notes)

        self.buffer = []
        self.scan_idx = 0
        self.part_idx = 0
        self.total_written = 0

        self.create_subscription(LaserScan, topic, self.on_scan, qos_profile_sensor_data)
        self.create_timer(flush_s, self.flush)
        self.get_logger().info(f"Logging {topic} -> {self.lidar_dir} (session {self.session_id})")

    # ---------- SQLite ----------
    def _init_db(self, notes):
        self.db = sqlite3.connect(os.path.join(self.output_dir, "sme.db"))
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute(
            """CREATE TABLE IF NOT EXISTS sessions (
                   session_id TEXT PRIMARY KEY,
                   start_wall_ns INTEGER,
                   end_wall_ns INTEGER,
                   sensor TEXT,
                   scan_count INTEGER,
                   notes TEXT)"""
        )
        self.db.execute(
            "INSERT INTO sessions (session_id, start_wall_ns, sensor, scan_count, notes) VALUES (?,?,?,?,?)",
            (self.session_id, time.time_ns(), "lidar", 0, notes),
        )
        self.db.commit()

    # ---------- ROS callback ----------
    def on_scan(self, msg: LaserScan):
        stamp = msg.header.stamp
        self.buffer.append({
            "session_id": self.session_id,
            "scan_idx": self.scan_idx,
            "t_mono_ns": time.monotonic_ns(),
            "t_wall_ns": time.time_ns(),
            "t_msg_ns": stamp.sec * 1_000_000_000 + stamp.nanosec,
            "angle_min": msg.angle_min,
            "angle_max": msg.angle_max,
            "angle_increment": msg.angle_increment,
            "range_min": msg.range_min,
            "range_max": msg.range_max,
            "scan_time": msg.scan_time,
            "ranges": list(msg.ranges),          # inf/NaN = no return; kept raw
            "intensities": list(msg.intensities),
        })
        self.scan_idx += 1

    # ---------- Parquet flush ----------
    def flush(self):
        if not self.buffer:
            return
        rows, self.buffer = self.buffer, []
        table = pa.Table.from_pylist(rows, schema=LIDAR_SCHEMA)
        final = os.path.join(self.lidar_dir, f"part-{self.part_idx:05d}.parquet")
        tmp = final + ".tmp"
        pq.write_table(table, tmp, compression="zstd")
        os.replace(tmp, final)                   # atomic: no half-written files
        self.part_idx += 1
        self.total_written += len(rows)
        self.db.execute(
            "UPDATE sessions SET scan_count=?, end_wall_ns=? WHERE session_id=?",
            (self.total_written, time.time_ns(), self.session_id),
        )
        self.db.commit()
        self.get_logger().info(f"flushed {len(rows)} scans (total {self.total_written})")

    def close(self):
        self.flush()
        self.db.close()


def main():
    rclpy.init()
    node = LidarLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
