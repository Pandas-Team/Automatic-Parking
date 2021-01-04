import math
import numpy as np

def angle_of_line(x1, y1, x2, y2):
    return math.atan2(y2-y1, x2-x1)