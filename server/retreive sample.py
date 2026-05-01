def retrieve_sample(distance_cm, mbot, flash_led,
                    approach_buffer_cm=5.0,
                    flash_times=5,
                    red=255,
                    green=0,
                    blue=0,
                    delay=0.3):
    """
    Approach the RED sample and trigger a visual cue.

    Returns the forward distance used to approach the sample.
    """
    approach_distance = 0
    if distance_cm > approach_buffer_cm:
        approach_distance = distance_cm - approach_buffer_cm
        mbot.straight(approach_distance)

    flash_led(flash_times, red, green, blue, delay)
    return approach_distance
