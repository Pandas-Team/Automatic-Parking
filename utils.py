import math
import numpy as np

def angle_of_line(x1, y1, x2, y2):
    return math.atan2(y2-y1, x2-x1)

def get_planning_points(end):
    s = 4
    l = 8
    d = 2
    w = 4
    x_ensure2 = end[0]
    y_ensure2 = end[1]
    x_ensure1 = x_ensure2 - d - w
    y_ensure1 = y_ensure2 + l + s

    ensure_path1 = np.vstack([np.repeat(x_ensure1,4/0.25), np.arange(y_ensure1,y_ensure1+4,0.25)]).T
    ensure_path2 = np.vstack([np.repeat(x_ensure2,4/0.25), np.arange(y_ensure2-4,y_ensure2,0.25)]).T
    
    return x_ensure1, y_ensure1, x_ensure2, y_ensure2, ensure_path1, ensure_path2
