import time


def push_movable_object(distance_cm, mbot2, turn, move_and_turn,
                        push_speed=40, turn_diff=20, push_time=4.0):
    """
    Move a GREEN obstacle out of the path.

    Returns True when a push sequence was executed, otherwise False.
    """
    if distance_cm <= 0 or distance_cm > 15:
        return False

    turn(180)
    mbot2.straight(-(distance_cm * 1.3))
    move_and_turn(speed=push_speed, diff=turn_diff, is_left=True)
    time.sleep(push_time)
    mbot2.straight(distance_cm * 1.3)
    turn(180)
    return True
