def begin_return_to_insertion(mbot):
    """
    Turn the robot around once after the sample is collected.
    """
    mbot.turnLeft(180)


def insertion_point_reached(color, insertion_color="YELLOW"):
    """
    Returns True when the camera sees the insertion point marker.
    """
    return color == insertion_color


def handle_return_obstacle(color, push_object_callback, steer_around_callback):
    """
    Decide how to handle an obstacle while returning to base.
    """
    if color == "GREEN":
        push_object_callback()
    else:
        steer_around_callback()
