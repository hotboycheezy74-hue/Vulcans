"""
Reference copy of the maze line-follow behavior.

This mirrors the active LINE_FOLLOW server behavior so the team has a
single-purpose file to read while tuning sensor-state handling.
"""

LINE_FULL_BLACK_STATUS = 15
LINE_RIGHT_RECOVERY_LEFT = 42
LINE_RIGHT_RECOVERY_RIGHT = 14
LINE_STRAIGHT_SPEED = 14
LINE_LEFT_CORRECTION_LEFT = 20
LINE_LEFT_CORRECTION_RIGHT = -22
LINE_HARD_LEFT_LEFT = -12
LINE_HARD_LEFT_RIGHT = -32
LINE_HARDEST_LEFT_LEFT = -24
LINE_HARDEST_LEFT_RIGHT = -40
LINE_RIGHT_TURN_LEFT = 46
LINE_RIGHT_TURN_RIGHT = 16
LINE_SETTLE_CYCLES = 2
LINE_SETTLE_LEFT = 16
LINE_SETTLE_RIGHT = -14
line_last_status = 1
line_right_turn_mode = False
line_settle_cycles = 0


def line_follow_behavior():
    global line_last_status, line_right_turn_mode, line_settle_cycles

    if not arbiter.acquire("line", "LINE_FOLLOW", 150, blocking=False):
        return
    try:
        status = mbuild.quad_rgb_sensor.get_line_sta()
    finally:
        arbiter.release("line", "LINE_FOLLOW")

    if not arbiter.acquire("motors", "LINE_FOLLOW", 150, blocking=False):
        return
    try:
        if status == 0 and line_last_status == 1:
            line_right_turn_mode = True

        if line_right_turn_mode:
            if status > 0:
                line_right_turn_mode = False
                line_settle_cycles = LINE_SETTLE_CYCLES
            else:
                mbot2.drive_speed(LINE_RIGHT_TURN_LEFT, LINE_RIGHT_TURN_RIGHT)
                line_last_status = status
                return

        if line_settle_cycles > 0:
            mbot2.drive_speed(LINE_SETTLE_LEFT, LINE_SETTLE_RIGHT)
            line_settle_cycles -= 1
            line_last_status = status
            return

        if status == 0:
            mbot2.drive_speed(LINE_RIGHT_RECOVERY_LEFT, LINE_RIGHT_RECOVERY_RIGHT)
        elif status == 1:
            mbot2.drive_speed(LINE_STRAIGHT_SPEED, -LINE_STRAIGHT_SPEED)
        elif status == 2 or status == 3:
            mbot2.drive_speed(LINE_LEFT_CORRECTION_LEFT, LINE_LEFT_CORRECTION_RIGHT)
        elif status < 7:
            mbot2.drive_speed(LINE_HARD_LEFT_LEFT, LINE_HARD_LEFT_RIGHT)
        else:
            if status >= LINE_FULL_BLACK_STATUS:
                mbot2.drive_speed(0, 0)
                time.sleep(0.01)
            mbot2.drive_speed(LINE_HARDEST_LEFT_LEFT, LINE_HARDEST_LEFT_RIGHT)

        line_last_status = status
    finally:
        arbiter.release("motors", "LINE_FOLLOW")
