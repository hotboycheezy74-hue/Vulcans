def avoid_immovable_object(distance_cm, mbot2, move_and_turn,
                           threshold_cm=25, speed=40, diff=20):
    """
    Steer around a BLUE obstacle.

    Returns "clear" if the path is still open and "avoided" when the
    robot performed an avoidance maneuver.
    """
    if distance_cm > threshold_cm:
        mbot2.drive_speed(speed, -speed)
        return "clear"

    if distance_cm <= threshold_cm * 0.6:
        turn_speed = max(25, speed - 5)
        turn_diff = min(turn_speed - 5, diff + 8)
        move_and_turn(speed=turn_speed, diff=turn_diff, is_left=False)
    else:
        turn_diff = max(10, diff - 8)
        move_and_turn(speed=speed, diff=turn_diff, is_left=False)
    return "avoided"
