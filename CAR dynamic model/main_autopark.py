import cv2
import numpy as np
from time import sleep

from environment import Environment, Parking1
from pathplanning import PathPlanning, ParkPathPlanning
from control import Car_Dynamics, MPC_Controller
from utils import angle_of_line

########################## default variables ################################################

start = np.array([0,90])
end   = np.array([90,80])

#############################################################################################

# environment margin  : 5
# pathplanning margin : 5

########################## defining obstacles ###############################################
obs1 = np.array([[30,i] for i in range(-5,80)] + [[70,i] for i in range(20,105)]) 
obs2 = np.array([[i,50] for i in range(50,70)]) 

# obs = np.array([[30,i] for i in range(30,70)] + [[70,i] for i in range(30,70)] + [[i,30] for i in range(30,70)] + [[i,70] for i in range(30,70)]) 
obs = np.vstack([obs1,obs2])

parking1 = Parking1(6)
end, obs = parking1.generate_obstacles()

# new_obs = np.array([[78,78],[79,79],[78,79]])
# obs = np.vstack([obs,new_obs])
#########################################################################################

########################### initialization ##############################################
env = Environment(obs)
my_car = Car_Dynamics(start[0], start[1], 0, np.deg2rad(0), length=4, dt=0.2)
controller = MPC_Controller(horiz=5)

res = env.render(my_car.x, my_car.y, my_car.phi, 0)
cv2.imshow('environment', res)
key = cv2.waitKey(1)
#############################################################################################

############################# path planning #################################################
park_path_planner = ParkPathPlanning(obs)
path_planner = PathPlanning(obs)

print('planning park scenario ...')
new_end, park_path, ensure_path1, ensure_path2 = park_path_planner.generate_park_behavior(int(start[0]),int(start[1]),int(end[0]),int(end[1]))
print('routing to destination ...')
path = path_planner.plan_path(int(start[0]),int(start[1]),int(new_end[0]),int(new_end[1]))

path = np.vstack([path, ensure_path1])
interpolated_path = path_planner.interpolate_path(path)

print('interpolating ...')
interpolated_park_path = park_path_planner.interpolate_park_path(park_path)
interpolated_park_path = np.vstack([ensure_path1[::-1], interpolated_park_path, ensure_path2[::-1]])

env.draw_path(interpolated_path)
env.draw_path(interpolated_park_path)
#############################################################################################

################################## control ##################################################

print('driving to destination ...')
for i,point in enumerate(interpolated_path):
        try:
            computed_angle = angle_of_line(interpolated_path[i][0],interpolated_path[i][1],interpolated_path[i+10][0],interpolated_path[i+10][1])
        except:
            pass
        
        acc, delta = controller.optimize(my_car,point[0],point[1],1,computed_angle)
        my_car.update_state(my_car.move(acc,  delta))
        res = env.render(my_car.x, my_car.y, my_car.phi, delta)
        cv2.imshow('environment', res)
        key = cv2.waitKey(1)
        if key == ord('s'):
            cv2.imwrite('res.png', res*255)


sleep(2)

for i,point in enumerate(interpolated_park_path):
        try:
            computed_angle = -angle_of_line(interpolated_park_path[i][0],interpolated_park_path[i][1],interpolated_park_path[i+10][0],interpolated_park_path[i+10][1])
        except:
            pass
        
        acc, delta = controller.optimize(my_car,point[0],point[1],-0.1,computed_angle)
        my_car.update_state(my_car.move(acc,  delta))
        res = env.render(my_car.x, my_car.y, my_car.phi, delta)
        cv2.imshow('environment', res)
        key = cv2.waitKey(1)
        if key == ord('s'):
            cv2.imwrite('res.png', res*255)


for i,point in enumerate(ensure_path2):
        try:
            computed_angle = angle_of_line(ensure_path2[i][0],ensure_path2[i][1],ensure_path2[i+10][0],ensure_path2[i+10][1])
        except:
            pass
        
        acc, delta = controller.optimize(my_car,point[0],point[1],0.1,computed_angle)
        my_car.update_state(my_car.move(acc,  delta))
        res = env.render(my_car.x, my_car.y, my_car.phi, delta)
        cv2.imshow('environment', res)
        key = cv2.waitKey(1)
        if key == ord('s'):
            cv2.imwrite('res.png', res*255)

# zeroing car steer
res = env.render(my_car.x, my_car.y, my_car.phi, 0)
cv2.imshow('environment', res)
key = cv2.waitKey(1)

sleep(10)

#############################################################################################


cv2.destroyAllWindows()

