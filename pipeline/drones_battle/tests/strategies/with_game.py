def compute_commands(my_team, enemy_team, target_pos, current_time, game=None):
    assert game is not None and game["max_speed"] > 0
    return {i: (0.0, 0.0, 0.0) for i in my_team}
