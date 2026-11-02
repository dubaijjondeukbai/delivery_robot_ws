"""
Pi 에 연결한 모니터에 QR 배달 미션 상태를 크게 띄운다 (pygame, 데스크톱 없이도 동작).

    ros2 run robot_bringup delivery_display
    ros2 launch robot_bringup delivery_demo.launch.py esp32_port:=... display:=true

필요: sudo apt install python3-pygame fonts-nanum   (한글 폰트가 없으면 글자가 네모로 나온다)
Ubuntu Server(데스크톱 없음): 콘솔 화면에 바로 그린다(SDL kmsdrm). 사용자가 video 그룹이어야 한다.
데스크톱이 있으면 그 화면에 전체 화면 창으로 뜬다(SSH 로 띄울 때는 DISPLAY=:0).
Esc 또는 q 키로 끈다.

입력: /delivery/state (delivery_demo), /safety/state, /vision/qr_result
문구는 display_text.py.
"""

from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
import time
from typing import List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_bringup.display_text import describe, Screen
from std_msgs.msg import String

FONT_CANDIDATES = (
    '/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf',
    '/usr/share/fonts/truetype/nanum/NanumGothic.ttf',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/noto-cjk/NotoSansCJK-Bold.ttc',
)

BACKGROUND = {
    'idle': (31, 42, 58),
    'move': (29, 78, 216),
    'wait': (15, 118, 110),
    'action': (180, 83, 9),
    'done': (21, 128, 61),
    'error': (185, 28, 28),
    'offline': (55, 65, 81),
}
BANNER = {'error': ((220, 38, 38), (255, 255, 255)), 'warn': ((250, 204, 21), (17, 24, 39))}
WHITE = (255, 255, 255)


def find_font(path: str = '') -> Optional[str]:
    if path:
        return path if os.path.isfile(path) else None
    for candidate in FONT_CANDIDATES:
        if os.path.isfile(candidate):
            return candidate
    if shutil.which('fc-match'):
        try:
            found = subprocess.run(['fc-match', '-f', '%{file}', ':lang=ko:weight=bold'],
                                   capture_output=True, text=True, timeout=5).stdout.strip()
            if found and os.path.isfile(found):
                return found
        except (OSError, subprocess.SubprocessError):
            pass
    return None


class Renderer:
    """pygame 화면. 내용이 바뀌거나 1 초가 지날 때만 다시 그린다."""

    def __init__(self, fullscreen: bool, size: List[int], font_path: Optional[str]) -> None:
        os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')
        import pygame
        self.pg = pygame
        pygame.display.init()
        pygame.font.init()
        if fullscreen:
            self.surface = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        else:
            self.surface = pygame.display.set_mode((int(size[0]), int(size[1])))
        pygame.display.set_caption('delivery_display')
        pygame.mouse.set_visible(False)
        self.width, self.height = self.surface.get_size()
        self.font_path = font_path
        self.fonts = {}

    def font(self, ratio: float):
        size = max(12, int(self.height * ratio))
        if size not in self.fonts:
            self.fonts[size] = self.pg.font.Font(self.font_path, size)
        return self.fonts[size]

    def wrap(self, text: str, font, width: int) -> List[str]:
        lines: List[str] = []
        current = ''
        for word in text.split(' '):
            trial = f'{current} {word}'.strip()
            if font.size(trial)[0] <= width:
                current = trial
                continue
            if current:
                lines.append(current)
            current = ''
            for char in word:  # 한 단어가 너무 길면 글자 단위로 자른다
                if font.size(current + char)[0] > width and current:
                    lines.append(current)
                    current = ''
                current += char
        if current:
            lines.append(current)
        return lines

    def text_block(self, text: str, ratio: float, top: int, color=WHITE,
                   max_lines: int = 3) -> int:
        font = self.font(ratio)
        margin = int(self.width * 0.05)
        lines = self.wrap(text, font, self.width - 2 * margin)[:max_lines]
        for line in lines:
            image = font.render(line, True, color)
            self.surface.blit(image, image.get_rect(midtop=(self.width // 2, top)))
            top += int(font.get_linesize() * 1.05)
        return top

    def draw(self, screen: Screen, clock: str) -> None:
        pg = self.pg
        width, height = self.width, self.height
        background = BACKGROUND.get(screen.tone, BACKGROUND['offline'])
        self.surface.fill(background)
        top = int(height * 0.06)
        if screen.banner:
            fill, ink = BANNER.get(screen.banner_tone, BANNER['warn'])
            band = int(height * 0.13)
            pg.draw.rect(self.surface, fill, (0, 0, width, band))
            font = self.font(0.06)
            lines = self.wrap(screen.banner, font, int(width * 0.92))[:1]
            image = font.render(lines[0] if lines else '', True, ink)
            self.surface.blit(image, image.get_rect(center=(width // 2, band // 2)))
            top = band + int(height * 0.04)

        top = max(top, int(height * 0.2))
        top = self.text_block(screen.title, 0.13, top, max_lines=2)
        top = self.text_block(screen.guide, 0.07, top + int(height * 0.04), max_lines=3)
        if screen.note:
            self.text_block(screen.note, 0.045, top + int(height * 0.03), (230, 230, 230), 2)

        bar = int(height * 0.1)
        shade = tuple(int(c * 0.6) for c in background)
        pg.draw.rect(self.surface, shade, (0, height - bar, width, bar))
        font = self.font(0.045)
        margin = int(width * 0.03)
        middle = height - bar // 2
        if screen.lid:
            image = font.render(screen.lid, True, WHITE)
            self.surface.blit(image, image.get_rect(midleft=(margin, middle)))
        right = font.render(clock, True, WHITE)
        self.surface.blit(right, right.get_rect(midright=(width - margin, middle)))
        if screen.qr:
            image = font.render(screen.qr, True, WHITE)
            self.surface.blit(image, image.get_rect(
                midright=(width - margin - right.get_width() - margin * 2, middle)))
        pg.display.flip()

    def keep_running(self) -> bool:
        for event in self.pg.event.get():
            if event.type == self.pg.QUIT:
                return False
            if event.type == self.pg.KEYDOWN and event.key in (self.pg.K_ESCAPE, self.pg.K_q):
                return False
        return True

    def save(self, path: str) -> None:
        self.pg.image.save(self.surface, path)

    def close(self) -> None:
        self.pg.quit()


class DisplayNode(Node):

    def __init__(self) -> None:
        super().__init__('delivery_display')
        defaults = {
            'state_topic': '/delivery/state',
            'safety_state_topic': '/safety/state',
            'qr_topic': '/vision/qr_result',
            'fullscreen': True,
            'window_size': [1024, 600],
            'font_path': '',        # 빈 값 = 나눔고딕 / Noto CJK 자동 검색
            'fps': 5.0,
            'stale_sec': 3.0,       # 이보다 오래 소식이 없으면 '연결 대기' / 'Safety 신호 없음'
            'screenshot_dir': '',   # 시험용: 화면이 바뀔 때마다 PNG 저장
        }
        self.p = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            self.p[name] = self.get_parameter(name).value
        self.snapshot: Optional[dict] = None
        self.snapshot_at = -math.inf
        self.safety = ''
        self.safety_at = -math.inf
        self.qr = ''
        self.qr_at = -math.inf
        self.create_subscription(String, self.p['state_topic'], self._on_state, 10)
        self.create_subscription(String, self.p['safety_state_topic'], self._on_safety, 10)
        self.create_subscription(String, self.p['qr_topic'], self._on_qr, 10)

    def _on_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(data, dict):
            self.snapshot, self.snapshot_at = data, time.monotonic()

    def _on_safety(self, msg: String) -> None:
        self.safety, self.safety_at = msg.data.strip(), time.monotonic()

    def _on_qr(self, msg: String) -> None:
        self.qr, self.qr_at = msg.data.strip()[:40], time.monotonic()

    def screen(self) -> Screen:
        now = time.monotonic()
        return describe(self.snapshot, now - self.snapshot_at, self.safety,
                        now - self.safety_at, self.qr, now - self.qr_at,
                        stale_sec=float(self.p['stale_sec']))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DisplayNode()
    renderer = None
    try:
        font = find_font(node.p['font_path'])
        if font is None:
            node.get_logger().warning(
                '[Display] 한글 폰트 없음 → 글자가 깨짐: sudo apt install fonts-nanum')
        try:
            renderer = Renderer(bool(node.p['fullscreen']), node.p['window_size'], font)
        except ImportError:
            node.get_logger().error('[Display] pygame 없음: sudo apt install python3-pygame')
            return
        except Exception as error:  # pygame.error: 화면 장치를 못 엶
            node.get_logger().error(
                f'[Display] 화면을 열 수 없음: {error}. 데스크톱이 없으면 video 그룹 확인'
                ' (sudo usermod -aG video $USER 후 다시 로그인), 데스크톱이면 DISPLAY=:0')
            return
        node.get_logger().info(
            f'[Display] {renderer.width}x{renderer.height}, font={font or "기본(한글 안 됨)"}')
        period = 1.0 / max(1.0, float(node.p['fps']))
        shot_dir = node.p['screenshot_dir']
        if shot_dir:
            os.makedirs(shot_dir, exist_ok=True)
        last = None
        last_content = None
        shots = 0
        while rclpy.ok() and renderer.keep_running():
            rclpy.spin_once(node, timeout_sec=period)
            screen = node.screen()
            clock = time.strftime('%H:%M:%S')
            if (screen, clock) == last:
                continue
            renderer.draw(screen, clock)
            last = (screen, clock)
            content = (screen.tone, screen.title, screen.guide, screen.banner, screen.lid)
            if shot_dir and content != last_content:
                shots += 1
                renderer.save(os.path.join(shot_dir, f'{shots:02d}_{screen.tone}.png'))
            last_content = content
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if renderer is not None:
            renderer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
