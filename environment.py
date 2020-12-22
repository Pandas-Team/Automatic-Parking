import cv2
import numpy as np
import scipy.interpolate as scipy_interpolate

class Environment():
    def __init__(self,obstacles):

        #coordinates are in [x,y] format
        self.car_length = 40
        self.car_width = 20
        self.margin = 3
        self.color = np.array([0,0,255])/255

        self.car_struct = np.array([[+self.car_length/2, +self.car_width/2],
                                    [+self.car_length/2, -self.car_width/2],  
                                    [-self.car_length/2, -self.car_width/2],
                                    [-self.car_length/2, +self.car_width/2]], 
                                    np.int32)

        #height and width
        self.background = np.ones((1000+20*self.margin,1000+20*self.margin,3))
        self.background[10:1000+20*self.margin:10,:] = np.array([200,200,200])/255
        self.background[:,10:1000+20*self.margin:10] = np.array([200,200,200])/255
        self.place_obstacles(obstacles)
                
    def place_obstacles(self, obs):
        obstacles = np.concatenate([np.array([[0,i] for i in range(100+2*self.margin)]),
                                    np.array([[100+2*self.margin-1,i] for i in range(100+2*self.margin)]),
                                    np.array([[i,0] for i in range(100+2*self.margin)]),
                                    np.array([[i,100+2*self.margin-1] for i in range(100+2*self.margin)]),
                                    obs + np.array([self.margin,self.margin])])*10
        for ob in obstacles:
            self.background[ob[1]:ob[1]+10,ob[0]:ob[0]+10]=0
    
    def draw_path(self, path):
        path = np.array(path)*10
        for p in path:
            self.background[p[1]+10*self.margin:p[1]+10*self.margin+10,p[0]+10*self.margin:p[0]+10*self.margin+10]=np.array([0,255,0])/255

    def interpolate_b_spline_path(self, x, y, n_path_points, degree=3):
        ipl_t = np.linspace(0.0, len(x) - 1, len(x))
        spl_i_x = scipy_interpolate.make_interp_spline(ipl_t, x, k=degree)
        spl_i_y = scipy_interpolate.make_interp_spline(ipl_t, y, k=degree)
        travel = np.linspace(0.0, len(x) - 1, n_path_points)
        return spl_i_x(travel), spl_i_y(travel)

    def interpolate_path(self, path):
        choices = np.arange(0,len(path),int(len(path)/32))
        way_point_x = path[choices,0]*10
        way_point_y = path[choices,1]*10
        n_course_point = 1000
        rix, riy = self.interpolate_b_spline_path(way_point_x, way_point_y, n_course_point)
        new_path = (np.vstack([rix,riy]).T).astype(int)
        for p in new_path:
            self.background[p[1]+10*self.margin:p[1]+10*self.margin+2,p[0]+10*self.margin:p[0]+10*self.margin+2]=np.array([255,0,0])/255


    def rotate_car(self, pts, degrees=0):
        angle = np.deg2rad(degrees)
        R = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle),  np.cos(angle)]])
        
        return ((R @ pts.T).T).astype(int)

    def render(self, x, y, angle):
        # x,y in 1000 coordinates
        rotated_struct = self.rotate_car(self.car_struct, degrees=angle)
        rotated_struct += np.array([x,y]) + np.array([10*self.margin,10*self.margin])
        rendered = cv2.fillPoly(self.background.copy(), [rotated_struct], self.color)
        rendered = cv2.resize(np.flip(rendered, axis=0), (700,700))
        return rendered
