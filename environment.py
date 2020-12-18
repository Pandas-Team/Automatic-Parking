import cv2
import numpy as np
import numpy.polynomial.polynomial as poly

class Environment():
    def __init__(self,obstacles):

        #coordinates are in [x,y] format
        self.car_length = 40
        self.car_width = 20
        self.margin = np.array([30,30])
        self.color = np.array([0,0,255])/255

        self.car_struct = np.array([[+self.car_length/2, +self.car_width/2],
                                    [+self.car_length/2, -self.car_width/2],  
                                    [-self.car_length/2, -self.car_width/2],
                                    [-self.car_length/2, +self.car_width/2]], 
                                    np.int32)

        #height and width
        self.background = np.ones((1060,1060,3))
        self.background[10:1060:10,:] = np.array([200,200,200])/255
        self.background[:,10:1060:10] = np.array([200,200,200])/255
        self.place_obstacles(obstacles)
                
    def place_obstacles(self, obs):
        obstacles = np.concatenate([np.array([[0,i] for i in range(106)]),
                                    np.array([[105,i] for i in range(106)]),
                                    np.array([[i,0] for i in range(106)]),
                                    np.array([[i,105] for i in range(106)]),
                                    obs])*10
        for ob in obstacles:
            self.background[ob[1]:ob[1]+10,ob[0]:ob[0]+10]=0
    
    def draw_path(self, path):
        path = np.array(path)*10
        for p in path:
            self.background[p[1]+30:p[1]+30+10,p[0]+30:p[0]+30+10]=np.array([0,255,0])/255

    def interpolate_path(self, path):
        path = np.array(path)*10
        coefs = poly.polyfit(path[:,0], path[:,1], 50)
        x_new = np.arange(1000)
        y_new = poly.polyval(x_new, coefs).astype(int)
        for p in zip(x_new,y_new):
            try:
                self.background[p[1]+30,p[0]+30]=np.array([255,0,0])/255
            except:
                pass

    def rotate_car(self, pts, degrees=0):
        angle = np.deg2rad(degrees)
        R = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle),  np.cos(angle)]])
        
        return ((R @ pts.T).T).astype(int)

    def render(self, x, y, angle):
        rotated_struct = self.rotate_car(self.car_struct, degrees=angle)
        rotated_struct += np.array([x,y]+self.margin)
        rendered = cv2.fillPoly(self.background.copy(), [rotated_struct], self.color)
        rendered = cv2.resize(np.flip(rendered, axis=0), (750,750))
        return rendered
