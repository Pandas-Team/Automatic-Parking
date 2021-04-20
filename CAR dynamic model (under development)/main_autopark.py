import cv2
import numpy as np
from time import sleep
import argparse

from environment import Environment, Parking1
from pathplanning import PathPlanning, ParkPathPlanning
from control import Car_Dynamics, MPC_Controller
from utils import angle_of_line, DataLogger


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--x_start', type=int, default=0, help='X of start')
    parser.add_argument('--y_start', type=int, default=90, help='Y of start')
    parser.add_argument('--x_end', type=int, default=90, help='X of end')
    parser.add_argument('--y_end', type=int, default=80, help='Y of end')
    parser.add_argument('--parking', type=int, default=2, help='park position in parking1 out of 24')

    args = parser.parse_args()
    logger = DataLogger()

    ########################## default variables ################################################

    start = np.array([args.x_start, args.y_start])
    end   = np.array([args.x_end, args.y_end])

    #############################################################################################

    # environment margin  : 5
    # pathplanning margin : 5

    ########################## defining obstacles ###############################################
    obs1 = np.array([[30,i] for i in range(-5,80)] + [[70,i] for i in range(20,105)]) 
    obs2 = np.array([[i,50] for i in range(50,70)]) 

    # obs = np.array([[30,i] for i in range(30,70)] + [[70,i] for i in range(30,70)] + [[i,30] for i in range(30,70)] + [[i,70] for i in range(30,70)]) 
    obs = np.vstack([obs1,obs2])

    parking1 = Parking1(args.parking)
    end, obs = parking1.generate_obstacles()

    # new_obs = np.array([[78,78],[79,79],[78,79]])
    # obs = np.vstack([obs,new_obs])
    #########################################################################################

    ########################### initialization ##############################################
    env = Environment(obs)
    my_car = Car_Dynamics(start[0], start[1], 0, 0, 0, 0, length=4, dt=0.17, Gama=0)
    MPC_HORIZON = 10
    controller = MPC_Controller()

    res = env.render(my_car.x, my_car.y, my_car.psi, 0)
    cv2.imshow('environment', res)
    key = cv2.waitKey(1)
    #############################################################################################

    ############################# path planning #################################################
    park_path_planner = ParkPathPlanning(obs)
    path_planner = PathPlanning(obs)

    print('planning park scenario ...')
    new_end, park_path, ensure_path1, ensure_path2 = park_path_planner.generate_park_scenario(int(start[0]),int(start[1]),int(end[0]),int(end[1]))
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

    # x = t
    # y = 50

    print('driving to destination ...')
    for i,point in enumerate(interpolated_path):
            
            acc, delta = controller.optimize(my_car, interpolated_path[i:i+MPC_HORIZON])
            my_car.update_state(my_car.move(acc,  delta))
            res = env.render(my_car.x, my_car.y, my_car.psi, delta)
            logger.log(point, my_car, acc, delta)
            cv2.imshow('environment', res)
            key = cv2.waitKey(1)
            if key == ord('s'):
                cv2.imwrite('res.png', res*255)


    sleep(2)

    for i,point in enumerate(interpolated_park_path):
            
            acc, delta = controller.optimize(my_car, interpolated_park_path[i:i+MPC_HORIZON])
            my_car.update_state(my_car.move(acc,  delta))
            res = env.render(my_car.x, my_car.y, my_car.psi, delta)
            logger.log(point, my_car, acc, delta)
            cv2.imshow('environment', res)
            key = cv2.waitKey(1)
            if key == ord('s'):
                cv2.imwrite('res.png', res*255)


    for i,point in enumerate(ensure_path2):
            
            acc, delta = controller.optimize(my_car, ensure_path2[i:i+MPC_HORIZON])
            my_car.update_state(my_car.move(acc,  delta))
            res = env.render(my_car.x, my_car.y, my_car.psi, delta)
            logger.log(point, my_car, acc, delta)
            cv2.imshow('environment', res)
            key = cv2.waitKey(1)
            if key == ord('s'):
                cv2.imwrite('res.png', res*255)

    # zeroing car steer
    res = env.render(my_car.x, my_car.y, my_car.psi, 0)
    logger.save_data()
    cv2.imshow('environment', res)
    key = cv2.waitKey(1)

    sleep(10)

    #############################################################################################

    cv2.destroyAllWindows()

