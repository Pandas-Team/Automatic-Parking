import cv2
import numpy as np
import time
import tqdm
import matplotlib.pyplot as plt

from environment import Environment
from pathplanning import PathPlanning

# environment margin  : 3
# pathplanning margin : 2

obs = np.array([[30,i] for i in range(-3,80)] + [[70,i] for i in range(20,103)]) 

env = Environment(obs)
car_center = np.array([0,0])
ang = 45

#path planning setup

path_planner = PathPlanning(obs)
path = path_planner.plan_path(0,0,90,90)

env.draw_path(path)
env.interpolate_path(path)
while True:
        
        res = env.render(car_center[0],car_center[1],ang)
        # cv2.imwrite('res.png', res*255)
        # break
        cv2.imshow('environment', res)
        key = cv2.waitKey(1)
        if key == ord('a'):
            car_center[0] -= 1
        if key == ord('d'):
            car_center[0] += 1
        if key == ord('w'):
            car_center[1] += 1
        if key == ord('s'):
            car_center[1] -= 1
        if key == ord('r'):
            ang += 1
        if key == ord('e'):
            ang -= 1
        if key == ord('q'):
            break
        print(car_center)

cv2.destroyAllWindows()

