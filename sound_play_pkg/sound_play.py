#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import signal
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor


class SoundPlayNode(Node):
    def __init__(self):
        super().__init__('sound_play')

        dyn = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter('topic', '', dyn)
        self.declare_parameter('play_mode', '', dyn)
        self.declare_parameter('file_ids', [], dyn)
        self.declare_parameter('file_names', [], dyn)
        # 선택 파라미터: 사운드 디렉토리 (지정 시 설치 share 대신 사용)
        self.declare_parameter('sounds_dir', '', dyn)

        topic = self.get_parameter('topic').get_parameter_value().string_value
        if not topic:
            self.get_logger().error("Required parameter 'topic' not set")
            raise RuntimeError("missing required parameter: topic")

        self.play_mode = self.get_parameter('play_mode').get_parameter_value().string_value
        if self.play_mode not in ('single', 'overlap'):
            self.get_logger().error("Parameter 'play_mode' must be 'single' or 'overlap'")
            raise RuntimeError("invalid or missing play_mode")

        # 사운드 디렉토리 결정
        sounds_param = self.get_parameter('sounds_dir').get_parameter_value().string_value
        if sounds_param:
            self.sounds_dir = Path(sounds_param).expanduser().resolve()
        else:
            pkg_share = get_package_share_directory('sound_play_pkg')
            self.sounds_dir = Path(pkg_share) / 'sound'
        if not self.sounds_dir.is_dir():
            self.get_logger().warn(f"sounds_dir not found: {self.sounds_dir}")

        # 매핑 생성 (없거나 오류면 예외)
        self.file_map = self._build_mapping()

        self.sub = self.create_subscription(UInt8, topic, self.on_msg, 10)
        self.get_logger().info(f"Listening on {topic}, sounds dir: {self.sounds_dir}")

        self.current_proc = None
        self.last_played_id = None

    def _build_mapping(self):
        ids_param = self.get_parameter('file_ids').value
        names_param = self.get_parameter('file_names').value
        if not isinstance(ids_param, (list, tuple)) \
        or not isinstance(names_param, (list, tuple)) \
        or len(ids_param) == 0 \
        or len(ids_param) != len(names_param):
            self.get_logger().error(
                "file_ids / file_names must be same-length non-empty lists (provided lengths: "
                f"{len(ids_param) if isinstance(ids_param,(list,tuple)) else 'N/A'} / "
                f"{len(names_param) if isinstance(names_param,(list,tuple)) else 'N/A'})"
            )
            raise RuntimeError("invalid file_ids/file_names parameters")
        mapping = {}
        for i, n in zip(ids_param, names_param):
            try:
                mapping[int(i)] = str(n)
            except Exception:
                continue
        if not mapping:
            self.get_logger().error("Resulting file mapping empty")
            raise RuntimeError("empty mapping after processing parameters")
        return mapping

    def on_msg(self, msg: UInt8):
        val = int(msg.data)
        # 동일 ID 반복 수신: 새 ID가 들어올 때까지 무시
        if self.last_played_id == val:
            return
        fname = self.file_map.get(val)
        if not fname:
            self.get_logger().warn(f"No file mapped for value={val}")
            # 매핑되지 않은 값도 반복 로그 방지를 위해 기록
            self.last_played_id = val
            return

        fpath = (self.sounds_dir / fname).resolve()
        if not fpath.exists():
            self.get_logger().error(f"File not found: {fpath}")
            return

        self.get_logger().info(f"Play({val}) -> {fpath.name}")

        # 확장자 기준 플레이어 선택
        ext = fpath.suffix.lower()
        if ext == '.wav':
            cmd = ['aplay', '-q', str(fpath)]
        elif ext == '.mp3':
            cmd = ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', str(fpath)]
        else:
            cmd = ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', str(fpath)]

        if self.play_mode == 'single':
            # 기존 재생 중이면 중단
            if self.current_proc and self.current_proc.poll() is None:
                try:
                    self.current_proc.send_signal(signal.SIGTERM)
                except Exception:
                    pass
            self.current_proc = subprocess.Popen(cmd)
        else:  # overlap
            subprocess.Popen(cmd)

        # 현재 ID를 마지막 재생 ID로 기록
        self.last_played_id = val

def main():
    rclpy.init()
    node = SoundPlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted (Ctrl-C)')
    finally:
        # 재생 중 프로세스 종료
        if node.current_proc and node.current_proc.poll() is None:
            try:
                node.current_proc.send_signal(signal.SIGTERM)
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
