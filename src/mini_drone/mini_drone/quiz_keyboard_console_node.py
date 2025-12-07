#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quiz Keyboard Console Node

Frontend의 CommandPanel과 동일한 기능을 터미널에서 키보드로 제어합니다.

Publishes:
    /quiz/command (std_msgs/String) - 퀴즈 명령 (start, detect, answer_correct, finish, emergency)
    /quiz/answer (std_msgs/String) - Mock 답변 (1, 2)
    /anafi/yolo/ocr_enable (std_msgs/Bool) - OCR 토글

Subscribes:
    /quiz/state (std_msgs/String) - 현재 퀴즈 상태
"""

import sys
import termios
import tty
import select
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Bool


class QuizKeyboardConsole(Node):
    """
    Quiz Demo 키보드 콘솔 노드

    키 매핑:
      s: Setup (start) - 드론 이륙
      d: Detect - 감지 시작
      o: OCR Toggle - OCR 활성화/비활성화
      1: Mock Answer 1
      2: Mock Answer 2
      c: Correct (answer_correct) - 정답 확인
      f: Finish - 드론 착륙
      e: Emergency Stop - 긴급 정지
      h: Help - 도움말 출력
      q: Quit - 종료
    """

    def __init__(self):
        super().__init__('quiz_keyboard_console')

        # ---- QoS ----
        qos_reliable = QoSProfile(depth=10)
        qos_reliable.reliability = ReliabilityPolicy.RELIABLE
        qos_reliable.history = HistoryPolicy.KEEP_LAST

        qos_best_effort = QoSProfile(depth=10)
        qos_best_effort.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_best_effort.history = HistoryPolicy.KEEP_LAST

        # ---- State ----
        self._lock = threading.Lock()
        self._current_state = 'UNKNOWN'
        self._ocr_enabled = False
        self._running = True

        # ---- Publishers ----
        self.pub_command = self.create_publisher(
            String, 'quiz/command', qos_reliable
        )
        self.pub_answer = self.create_publisher(
            String, 'quiz/answer', qos_reliable
        )
        self.pub_ocr_enable = self.create_publisher(
            Bool, 'anafi/yolo/ocr_enable', qos_reliable
        )

        # ---- Subscribers ----
        self.sub_state = self.create_subscription(
            String, 'quiz/state', self._state_cb, qos_best_effort
        )

        # ---- Status Timer (1Hz) ----
        self.create_timer(1.0, self._status_timer)

        # ---- Keyboard Thread ----
        self._key_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._key_thread.start()

        self._print_header()
        self._print_help()

    # ---------- Callbacks ----------
    def _state_cb(self, msg: String):
        with self._lock:
            old_state = self._current_state
            self._current_state = msg.data
            
            # State 변경 시 OCR 상태 리셋
            if old_state == 'DETECTING' and self._current_state != 'DETECTING':
                self._ocr_enabled = False

    # ---------- Publishers ----------
    def _publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.pub_command.publish(msg)
        self._log_action(f'Command: {command}')

    def _publish_answer(self, answer: str):
        msg = String()
        msg.data = answer
        self.pub_answer.publish(msg)
        self._log_action(f'Mock Answer: {answer}')

    def _toggle_ocr(self):
        with self._lock:
            self._ocr_enabled = not self._ocr_enabled
            new_state = self._ocr_enabled
        
        msg = Bool()
        msg.data = new_state
        self.pub_ocr_enable.publish(msg)
        self._log_action(f'OCR: {"ENABLED" if new_state else "DISABLED"}')

    # ---------- Helpers ----------
    def _log_action(self, action: str):
        timestamp = datetime.now().strftime('%H:%M:%S')
        state = self._get_state()
        print(f'\r\033[K[{timestamp}] [{state:10}] → {action}')
        self._print_prompt()

    def _get_state(self) -> str:
        with self._lock:
            return self._current_state

    def _get_ocr_enabled(self) -> bool:
        with self._lock:
            return self._ocr_enabled

    def _status_timer(self):
        """주기적으로 상태 출력"""
        pass  # 상태는 prompt에서 보여줌

    def _print_prompt(self):
        """현재 상태와 가능한 명령 표시"""
        state = self._get_state()
        ocr = self._get_ocr_enabled()
        
        # 상태별 활성화된 명령 표시
        available = []
        if state in ['UNINIT', 'UNKNOWN']:
            available.append('[s]Setup')
        if state == 'IDLE':
            available.append('[d]Detect')
        if state == 'DETECTING':
            available.append(f'[o]OCR{"(ON)" if ocr else ""}')
            available.append('[1][2]Mock')
        if state == 'DRAWING':
            available.append('[c]Correct')
        if state in ['IDLE', 'DETECTING', 'DRAWING']:
            available.append('[f]Finish')
        if state != 'FINISH':
            available.append('[e]E-STOP')
        
        available_str = ' '.join(available) if available else 'No commands available'
        
        # ANSI escape로 프롬프트 출력
        print(f'\r\033[K\033[1;36m[{state}]\033[0m {available_str} > ', end='', flush=True)

    def _can_execute(self, command: str) -> bool:
        """현재 상태에서 명령 실행 가능 여부 확인"""
        state = self._get_state()
        
        if command == 'start':
            return state in ['UNINIT', 'UNKNOWN']
        elif command == 'detect':
            return state == 'IDLE'
        elif command == 'ocr':
            return state == 'DETECTING'
        elif command == 'mock':
            return state == 'DETECTING'
        elif command == 'answer_correct':
            return state == 'DRAWING'
        elif command == 'finish':
            return state in ['IDLE', 'DETECTING', 'DRAWING']
        elif command == 'emergency':
            return state != 'FINISH'
        
        return False

    # ---------- Keyboard ----------
    def _get_key(self, timeout=0.1) -> str | None:
        """non-blocking key read (Linux)"""
        fd = sys.stdin.fileno()
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None

    def _keyboard_loop(self):
        """키보드 입력 루프"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setcbreak(fd)
            self._print_prompt()
            
            while rclpy.ok() and self._running:
                key = self._get_key(timeout=0.1)
                if key is None:
                    continue
                
                if key == '\x03':  # Ctrl+C
                    self._running = False
                    break
                
                self._handle_key(key)
        
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _handle_key(self, key: str):
        key = key.lower()
        
        # Setup (start)
        if key == 's':
            if self._can_execute('start'):
                self._publish_command('start')
            else:
                self._log_action('Cannot Setup in current state')
            return
        
        # Detect
        if key == 'd':
            if self._can_execute('detect'):
                self._publish_command('detect')
            else:
                self._log_action('Cannot Detect in current state')
            return
        
        # OCR Toggle
        if key == 'o':
            if self._can_execute('ocr'):
                self._toggle_ocr()
            else:
                self._log_action('OCR only available in DETECTING state')
            return
        
        # Mock Answer 1
        if key == '1':
            if self._can_execute('mock'):
                self._publish_answer('1')
            else:
                self._log_action('Mock answer only available in DETECTING state')
            return
        
        # Mock Answer 2
        if key == '2':
            if self._can_execute('mock'):
                self._publish_answer('2')
            else:
                self._log_action('Mock answer only available in DETECTING state')
            return
        
        # Correct (answer_correct)
        if key == 'c':
            if self._can_execute('answer_correct'):
                self._publish_command('answer_correct')
            else:
                self._log_action('Cannot Correct in current state')
            return
        
        # Finish
        if key == 'f':
            if self._can_execute('finish'):
                self._publish_command('finish')
            else:
                self._log_action('Cannot Finish in current state')
            return
        
        # Emergency Stop
        if key == 'e':
            if self._can_execute('emergency'):
                self._publish_command('emergency')
            else:
                self._log_action('Emergency not available in FINISH state')
            return
        
        # Help
        if key == 'h':
            print()  # 새 줄
            self._print_help()
            return
        
        # Quit
        if key == 'q':
            self._running = False
            print('\n\033[1;33mQuitting...\033[0m')
            return

    def _print_header(self):
        header = """
\033[1;35m╔═══════════════════════════════════════════════════════════╗
║            🎮 Quiz Keyboard Console 🎮                    ║
║         (Frontend CommandPanel 터미널 버전)                ║
╚═══════════════════════════════════════════════════════════╝\033[0m
"""
        print(header)

    def _print_help(self):
        help_text = """
\033[1;33m═══════════════════════ CONTROLS ═══════════════════════\033[0m

  \033[1;32m[s]\033[0m Setup     - 드론 이륙 (UNINIT → IDLE)
  \033[1;32m[d]\033[0m Detect    - 감지 시작 (IDLE → DETECTING)
  \033[1;32m[o]\033[0m OCR       - OCR 활성화/비활성화 토글
  \033[1;32m[1]\033[0m Mock 1    - Mock 답변 1 전송
  \033[1;32m[2]\033[0m Mock 2    - Mock 답변 2 전송
  \033[1;32m[c]\033[0m Correct   - 정답 확인 (DRAWING → IDLE)
  \033[1;32m[f]\033[0m Finish    - 드론 착륙 (→ FINISH)
  \033[1;31m[e]\033[0m E-STOP    - 긴급 정지

  \033[1;36m[h]\033[0m Help      - 도움말 표시
  \033[1;36m[q]\033[0m Quit      - 종료

\033[1;33m═══════════════════════ STATE FLOW ═══════════════════════\033[0m

  UNINIT ──[s]──▶ IDLE ──[d]──▶ DETECTING ──[answer]──▶ DRAWING
                   ▲                                      │
                   └──────────────[c]─────────────────────┘

\033[1;33m══════════════════════════════════════════════════════════\033[0m
"""
        print(help_text)
        self._print_prompt()


def main(args=None):
    rclpy.init(args=args)
    node = QuizKeyboardConsole()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

