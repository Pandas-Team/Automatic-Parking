import cv2
import numpy as np
import time

from environment import Environment
from pathplanning import PathPlanning
from control import Car_Dynamics, MPC_Controller
from utils import angle_of_line

my_car = Car_Dynamics(0,0,0,np.deg2rad(45),4,0.2)
controller = MPC_Controller(horiz=5)

# environment margin  : 5
# pathplanning margin : 4

# obs1 = np.array([[30,i] for i in range(-5,80)] + [[70,i] for i in range(20,105)]) 
# obs2 = np.array([[i,50] for i in range(50,70)]) 

obs = np.array([[30,i] for i in range(30,70)] + [[70,i] for i in range(30,70)] + [[i,30] for i in range(30,70)] + [[i,70] for i in range(30,70)]) 
# obs = np.vstack([obs1,obs2])

# new_obs = np.array([[78,78],[79,79],[78,79]])
# obs = np.vstack([obs,new_obs])

env = Environment(obs)

#path planning setup

path_planner = PathPlanning(obs)
path = path_planner.plan_path(0,0,90,80)

s = 4
l = 8
d = 2
w = 4
x0 = 90 - d - w
y0 = 80 + l + s
ensure_path = np.vstack([np.repeat(x0,10), np.arange(y0,y0+5,0.5)]).T
ensure_path_2 = np.vstack([np.repeat(90,10), np.arange(80-5,80,0.5)]).T

path = np.vstack([path, ensure_path])

interpolated_path = path_planner.interpolate_path(path)
# print('path = \n',interpolated_path)

park_path = path_planner.plan_park_path(90,80)
# park_path = np.vstack([ensure_path[::-1], park_path, ensure_path_2[::-1]])
interpolated_park_path = path_planner.interpolate_park_path(park_path)
interpolated_park_path = np.vstack([ensure_path[::-1], interpolated_park_path, ensure_path_2[::-1]])

# env.draw_path(ensure_path)
env.draw_path(interpolated_path)
env.draw_path(interpolated_park_path)


for i,point in enumerate(interpolated_path):
        try:
            computed_angle = angle_of_line(interpolated_path[i][0],interpolated_path[i][1],interpolated_path[i+10][0],interpolated_path[i+10][1])
        except:
            pass

        acc, delta = controller.optimize(my_car,point[0],point[1],1,computed_angle)
        my_car.update_state(my_car.move(acc,  delta))
        
        res = env.render(my_car.x, my_car.y, my_car.phi, delta)
        # cv2.imwrite('res.png', res*255)
        # break
        cv2.imshow('environment', res)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break


from time import sleep
sleep(2)
# env.draw_path(interpolated_park_path)

for i,point in enumerate(interpolated_park_path):
        try:
            computed_angle = angle_of_line(interpolated_path[i][0],interpolated_path[i][1],interpolated_path[i+10][0],interpolated_path[i+10][1])
        except:
            pass
        # print(point)

        acc, delta = controller.optimize(my_car,point[0],point[1],-0.1,computed_angle)
        my_car.update_state(my_car.move(acc,  delta))
        
        res = env.render(my_car.x, my_car.y, my_car.phi, delta)
        # cv2.imwrite('res.png', res*255)
        # break
        cv2.imshow('environment', res)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break


for i,point in enumerate(ensure_path_2):
        try:
            computed_angle = angle_of_line(interpolated_path[i][0],interpolated_path[i][1],interpolated_path[i+10][0],interpolated_path[i+10][1])
        except:
            pass
        # print(point)

        acc, delta = controller.optimize(my_car,point[0],point[1],0.1,computed_angle)
        my_car.update_state(my_car.move(acc,  delta))
        
        res = env.render(my_car.x, my_car.y, my_car.phi, delta)
        # cv2.imwrite('res.png', res*255)
        # break
        cv2.imshow('environment', res)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

res = env.render(my_car.x, my_car.y, my_car.phi, 0)
cv2.imshow('environment', res)
key = cv2.waitKey(1)
sleep(10)


cv2.destroyAllWindows()

