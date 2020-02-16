def clamping(range_min, range_max, value) -> float:
    return max(min(value, range_max), range_min)