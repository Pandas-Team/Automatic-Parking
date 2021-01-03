import cv2
import numpy as np
import time
import tqdm
import matplotlib.pyplot as plt

from environment import Environment
from pathplanning import PathPlanning
from control import *
from scipy.optimize import minimize

my_car = Car_Dynamics(0,0,0,np.deg2rad(45),4,0.5)
controller = MPC_Controller(horiz=5)

# environment margin  : 5
# pathplanning margin : 4

obs1 = np.array([[30,i] for i in range(-5,80)] + [[70,i] for i in range(20,105)]) 
obs2 = np.array([[i,50] for i in range(50,70)]) 

# obs2 = np.array([[45,i] for i in range(45,5)] + [[55,i] for i in range(45,55)] + [[i,45] for i in range(45,5)] + [[i,55] for i in range(45,55)]) 
obs = np.vstack([obs1,obs2])

# new_obs = np.array([[78,78],[79,79],[78,79]])
# obs = np.vstack([obs,new_obs])

env = Environment(obs)

#path planning setup

path_planner = PathPlanning(obs)
path = path_planner.plan_path(0,0,90,90)
print(path)
interpolated_path = path_planner.interpolate_path(path)

print('path = \n',interpolated_path)

env.draw_path(interpolated_path)

for i,point in enumerate(interpolated_path):
        try:
            computed_angle = np.arctan((interpolated_path[i+5][1]-interpolated_path[i][1])/(interpolated_path[i+5][0]-interpolated_path[i][0]))
        except:
            pass
        print(point)

        acc, delta = controller.optimize(my_car,point[0],point[1],1,computed_angle)
        my_car.update_state(my_car.move(acc,  delta))
        
        res = env.render(my_car.x, my_car.y, my_car.phi)
        # cv2.imwrite('res.png', res*255)
        # break
        cv2.imshow('environment', res)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        # print(car_center)

cv2.destroyAllWindows()

