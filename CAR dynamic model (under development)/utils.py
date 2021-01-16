import math
import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager


def angle_of_line(x1, y1, x2, y2):
    return math.atan2(y2-y1, x2-x1)

class DataLogger:
    def __init__(self):
        self.path = []
        self.car_state = []
        self.u = []

    def log(self, point, my_car, acc, delta):
        self.path.append(point)
        self.car_state.append([my_car.x, my_car.y, my_car.psi,my_car.u,my_car.v,my_car.r])
        self.u.append([acc, delta])

    def save_data(self):
        os.makedirs('log results', exist_ok=True)
        t = np.arange(0,len(self.path)/5,0.2)
        self.path = np.array(self.path)
        self.car_state = np.array(self.car_state)
        self.u = np.array(self.u)
        font = font_manager.FontProperties(family='Times New Roman', weight='bold',style='normal', size=20)

        # plot x
        plt.figure(figsize=(12,8))
        plt.scatter(t, self.car_state[:,0], color='r', s=8)
        plt.scatter(t, self.path[:,0], color='b', s=8)
        plt.title('car\'s x in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('x (m)',fontsize=20)
        plt.grid()
        plt.legend(['car\'s x', 'reference'], prop=font) # using a named size
        plt.savefig('log results/x.png')

        # plot y
        plt.figure(figsize=(12,8))
        plt.scatter(t, self.car_state[:,1], color='r', s=8)
        plt.scatter(t, self.path[:,1], color='b', s=8)
        plt.title('car\'s y in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('y (m)',fontsize=20)
        plt.grid()
        plt.legend(['car\'s y', 'reference'], prop=font) # using a named size
        plt.savefig('log results/y.png')

        # plot psi
        plt.figure(figsize=(12,8))
        plt.scatter(t, np.rad2deg(self.car_state[:,2]), color='r', s=8)
        plt.title('car\'s angle in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('psi (degree)',fontsize=20)
        plt.grid()
        plt.legend(['car angle (degree)'], prop=font) # using a named size
        plt.savefig('log results/psi.png')

        # plot u
        plt.figure(figsize=(12,8))
        plt.scatter(t, self.car_state[:,3], color='r', s=8)
        plt.title('car\'s u in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('u (m/s)',fontsize=20)
        plt.grid()
        plt.legend(['car u (m/s)'], prop=font) # using a named size
        plt.savefig('log results/u.png')

        # plot v
        plt.figure(figsize=(12,8))
        plt.scatter(t, self.car_state[:,4], color='r', s=8)
        plt.title('car\'s v in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('v (m/s)',fontsize=20)
        plt.grid()
        plt.legend(['car speed (m/s)'], prop=font) # using a named size
        plt.savefig('log results/v.png')

        # plot r
        plt.figure(figsize=(12,8))
        plt.scatter(t, self.car_state[:,5], color='r', s=8)
        plt.title('car\'s r in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('r (m/s)',fontsize=20)
        plt.grid()
        plt.legend(['car r (m/s)'], prop=font) # using a named size
        plt.savefig('log results/r.png')

        # plot position
        plt.figure(figsize=(12,12))
        plt.scatter(self.path[:,0], self.path[:,1], color='b', s=5)
        plt.scatter(self.car_state[:,0], self.car_state[:,1], color='r', s=5)
        plt.title('car\'s position in time',fontsize=20)
        plt.xlabel('x (m)',fontsize=20)
        plt.ylabel('y (m)',fontsize=20)
        plt.grid()
        plt.legend(['reference','car\'s position'], prop=font) # using a named size
        plt.savefig('log results/position.png')

        # plot accelerate
        plt.figure(figsize=(12,8))
        plt.scatter(t, self.u[:,0], color='r', s=8)
        plt.title('car\'s accelerate in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('accelerate (m^2/s)',fontsize=20)
        plt.grid()
        plt.legend(['car accelerate (m^2/s)'], prop=font) # using a named size
        plt.savefig('log results/accelerate.png')

        # plot delta
        plt.figure(figsize=(12,8))
        plt.scatter(t, np.rad2deg(self.u[:,1]), color='r', s=8)
        plt.title('car\'s steer in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('steer (degree)',fontsize=20)
        plt.grid()
        plt.legend(['car steer (degree)'], prop=font) # using a named size
        plt.savefig('log results/steer.png')

        print('all data saved on log results ...')
