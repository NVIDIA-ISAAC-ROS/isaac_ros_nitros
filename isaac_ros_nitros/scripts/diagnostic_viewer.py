#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import curses
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node


"""
Utility for viewing the nitros diagnostics in a terminal interface.

You can either use diagnostics that are built into applications, for example the
isaac_ros_data_recorder, or you can export ENABLE_GLOBAL_DIAGNOSTICS=1 and view much more
"""


class DiagnosticsDisplay(Node):

    def __init__(self):
        super().__init__('diagnostics_display')
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.listener_callback,
            10)
        self.subscription

        # Thread-safe storage for diagnostic data
        self.data_lock = threading.Lock()
        self.status_data = {}

    def listener_callback(self, msg):
        with self.data_lock:
            for status in msg.status:
                self.status_data[status.name] = status


def curses_main(stdscr, node):
    stdscr.nodelay(True)
    curses.curs_set(0)
    stdscr.keypad(True)  # Enable keypad mode to capture special keys

    start_idx = 0
    key = -1

    while rclpy.ok():
        stdscr.clear()
        # Get terminal dimensions
        height, width = stdscr.getmaxyx()

        MAX_NAME_WIDTH = 100
        FRAME_RATE_WIDTH = 15

        # Adjust column widths if terminal is too narrow
        total_width_needed = MAX_NAME_WIDTH + 2 * FRAME_RATE_WIDTH + 2  # +2 for spaces
        if total_width_needed > width:
            # Reduce the widths proportionally
            scaling_factor = width / total_width_needed
            MAX_NAME_WIDTH = int(MAX_NAME_WIDTH * scaling_factor)
            FRAME_RATE_WIDTH = int(FRAME_RATE_WIDTH * scaling_factor)

        header = (f"{'Topic Name':<{MAX_NAME_WIDTH}} {'frame_rate_msg':<{FRAME_RATE_WIDTH}}"
                  f"{'frame_rate_node':<{FRAME_RATE_WIDTH}}")
        separator = '-' * min(width, MAX_NAME_WIDTH + 2 * FRAME_RATE_WIDTH + 2)

        try:
            stdscr.addstr(0, 0, header[:width - 1])
            stdscr.addstr(1, 0, separator)
        except curses.error:
            pass  # Ignore if cannot write header (terminal too small)

        with node.data_lock:
            status_list = list(node.status_data.values())

        # Handle key presses
        try:
            key = stdscr.getch()
        except Exception:
            key = -1

        # Up and down arrow keys to scroll
        if key == curses.KEY_UP:
            start_idx = max(0, start_idx - 1)
        elif key == curses.KEY_DOWN:
            start_idx = min(len(status_list) - (height - 5), start_idx + 1)
        elif key == curses.KEY_PPAGE:  # Page Up
            start_idx = max(0, start_idx - (height - 5))
        elif key == curses.KEY_NPAGE:  # Page Down
            start_idx = min(len(status_list) - (height - 5), start_idx + (height - 5))
        elif key == ord('q') or key == ord('Q'):  # Quit on 'q' or 'Q'
            break

        # Ensure start_idx is within valid range
        if start_idx < 0:
            start_idx = 0
        max_start_idx = max(0, len(status_list) - (height - 5))
        if start_idx > max_start_idx:
            start_idx = max_start_idx

        visible_height = height - 5  # Adjusted for header, footer, and indicators

        visible_status_list = status_list[start_idx:start_idx + visible_height]

        for idx, status in enumerate(visible_status_list):
            name = status.name
            frame_rate_msg = ''
            frame_rate_node = ''
            # Extract frame rates from key-value pairs
            for kv in status.values:
                if kv.key == 'frame_rate_msg':
                    frame_rate_msg = kv.value
                if kv.key == 'frame_rate_node':
                    frame_rate_node = kv.value

            # No need to truncate the name since MAX_NAME_WIDTH is large
            name_display = name.ljust(MAX_NAME_WIDTH)

            # Format the data
            frame_rate_msg_display = frame_rate_msg.ljust(FRAME_RATE_WIDTH)
            frame_rate_node_display = frame_rate_node.ljust(FRAME_RATE_WIDTH)
            line = f'{name_display} {frame_rate_msg_display} {frame_rate_node_display}'

            # Truncate the line if necessary
            line = line[:width - 1]  # Reserve last character for cursor

            try:
                stdscr.addstr(idx + 2, 0, line)
            except curses.error:
                pass  # Ignore if cannot write line (terminal too small)

        # Add indicators if there's more data above or below
        try:
            if start_idx > 0:
                stdscr.addstr(2, width - 2, '↑')
            if start_idx + visible_height < len(status_list):
                stdscr.addstr(height - 3, width - 2, '↓')
        except curses.error:
            pass

        # Add a footer with status information
        status_line = (f'Showing {start_idx + 1} - {start_idx + len(visible_status_list)} of '
                       f'{len(status_list)} topics. Press "q" to quit.')

        try:
            stdscr.addstr(height - 2, 0, status_line[:width - 1])
        except curses.error:
            pass

        stdscr.refresh()
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsDisplay()

    try:
        thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        thread.start()
        curses.wrapper(curses_main, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
