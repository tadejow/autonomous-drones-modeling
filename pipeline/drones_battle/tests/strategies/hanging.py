import time


def compute_commands(my_team, enemy_team, target_pos, current_time):
    if current_time > 1.0:
        while True:
            time.sleep(0.1)
    return {i: (1.0, 0.0, 0.0) for i in my_team}
