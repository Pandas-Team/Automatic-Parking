import numpy as np
import math
import scipy.interpolate as scipy_interpolate
from utils import angle_of_line


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d < self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


class PathPlanning:
    def __init__(self,obstacles):
        self.margin = 5
        #sacale obstacles from env margin to pathplanning margin
        obstacles = obstacles + np.array([self.margin,self.margin])
        obstacles = obstacles[(obstacles[:,0]>=0) & (obstacles[:,1]>=0)]

        self.obs = np.concatenate([np.array([[0,i] for i in range(100+self.margin)]),
                                  np.array([[100+2*self.margin,i] for i in range(100+2*self.margin)]),
                                  np.array([[i,0] for i in range(100+self.margin)]),
                                  np.array([[i,100+2*self.margin] for i in range(100+2*self.margin)]),
                                  obstacles])

        self.ox = [int(item) for item in self.obs[:,0]]
        self.oy = [int(item) for item in self.obs[:,1]]
        self.grid_size = 1
        self.robot_radius = 4
        self.a_star = AStarPlanner(self.ox, self.oy, self.grid_size, self.robot_radius)

    def plan_path(self,sx, sy, gx, gy):    
        rx, ry = self.a_star.planning(sx+self.margin, sy+self.margin, gx+self.margin, gy+self.margin)
        rx = np.array(rx)-self.margin+0.5
        ry = np.array(ry)-self.margin+0.5
        path = np.vstack([rx,ry]).T
        return path[::-1]

    def interpolate_b_spline_path(self, x, y, n_path_points, degree=3):
        ipl_t = np.linspace(0.0, len(x) - 1, len(x))
        spl_i_x = scipy_interpolate.make_interp_spline(ipl_t, x, k=degree)
        spl_i_y = scipy_interpolate.make_interp_spline(ipl_t, y, k=degree)
        travel = np.linspace(0.0, len(x) - 1, n_path_points)
        return spl_i_x(travel), spl_i_y(travel)

    def interpolate_path(self, path):
        choices = np.arange(0,len(path),int(len(path)/32))
        way_point_x = path[choices,0]
        way_point_y = path[choices,1]
        n_course_point = int(len(choices)*18)
        rix, riy = self.interpolate_b_spline_path(way_point_x, way_point_y, n_course_point)
        new_path = np.vstack([rix,riy]).T
        # new_path[new_path<0] = 0
        return new_path





class ParkPathPlanning:
    def __init__(self,obstacles):
        self.margin = 5
        #sacale obstacles from env margin to pathplanning margin
        obstacles = obstacles + np.array([self.margin,self.margin])
        obstacles = obstacles[(obstacles[:,0]>=0) & (obstacles[:,1]>=0)]

        self.obs = np.concatenate([np.array([[0,i] for i in range(100+self.margin)]),
                                  np.array([[100+2*self.margin,i] for i in range(100+2*self.margin)]),
                                  np.array([[i,0] for i in range(100+self.margin)]),
                                  np.array([[i,100+2*self.margin] for i in range(100+2*self.margin)]),
                                  obstacles])

        self.ox = [int(item) for item in self.obs[:,0]]
        self.oy = [int(item) for item in self.obs[:,1]]
        self.grid_size = 1
        self.robot_radius = 4
        self.a_star = AStarPlanner(self.ox, self.oy, self.grid_size, self.robot_radius)

    def generate_park_scenario(self,sx, sy, gx, gy):    
        rx, ry = self.a_star.planning(sx+self.margin, sy+self.margin, gx+self.margin, gy+self.margin)
        rx = np.array(rx)-self.margin+0.5
        ry = np.array(ry)-self.margin+0.5
        path = np.vstack([rx,ry]).T
        path = path[::-1]
        computed_angle = angle_of_line(path[-10][0],path[-10][1],path[-1][0],path[-1][1])

        s = 4
        l = 8
        d = 2
        w = 4

        if -math.atan2(0,-1) < computed_angle <= math.atan2(-1,0):
            x_ensure2 = gx
            y_ensure2 = gy
            x_ensure1 = x_ensure2 + d + w
            y_ensure1 = y_ensure2 - l - s
            ensure_path1 = np.vstack([np.repeat(x_ensure1,4/0.25), np.arange(y_ensure1-4,y_ensure1,0.25)[::-1]]).T
            ensure_path2 = np.vstack([np.repeat(x_ensure2,4/0.25), np.arange(y_ensure2,y_ensure2+4,0.25)[::-1]]).T
            park_path = self.plan_park_down_right(x_ensure2, y_ensure2)

        elif math.atan2(-1,0) <= computed_angle <= math.atan2(0,1):
            x_ensure2 = gx
            y_ensure2 = gy
            x_ensure1 = x_ensure2 - d - w
            y_ensure1 = y_ensure2 - l - s 
            ensure_path1 = np.vstack([np.repeat(x_ensure1,4/0.25), np.arange(y_ensure1-4,y_ensure1,0.25)[::-1]]).T
            ensure_path2 = np.vstack([np.repeat(x_ensure2,4/0.25), np.arange(y_ensure2,y_ensure2+4,0.25)[::-1]]).T
            park_path = self.plan_park_down_left(x_ensure2, y_ensure2)

        elif math.atan2(0,1) < computed_angle <= math.atan2(1,0):
            x_ensure2 = gx
            y_ensure2 = gy
            x_ensure1 = x_ensure2 - d - w
            y_ensure1 = y_ensure2 + l + s
            ensure_path1 = np.vstack([np.repeat(x_ensure1,4/0.25), np.arange(y_ensure1,y_ensure1+4,0.25)]).T
            ensure_path2 = np.vstack([np.repeat(x_ensure2,4/0.25), np.arange(y_ensure2-4,y_ensure2,0.25)]).T
            park_path = self.plan_park_up_left(x_ensure2, y_ensure2)

        elif math.atan2(1,0) < computed_angle <= math.atan2(0,-1):
            x_ensure2 = gx
            y_ensure2 = gy
            x_ensure1 = x_ensure2 + d + w
            y_ensure1 = y_ensure2 + l + s
            ensure_path1 = np.vstack([np.repeat(x_ensure1,4/0.25), np.arange(y_ensure1,y_ensure1+4,0.25)]).T
            ensure_path2 = np.vstack([np.repeat(x_ensure2,4/0.25), np.arange(y_ensure2-4,y_ensure2,0.25)]).T
            park_path = self.plan_park_up_right(x_ensure2, y_ensure2)

        return np.array([x_ensure1, y_ensure1]), park_path, ensure_path1, ensure_path2

    def interpolate_b_spline_path(self, x, y, n_path_points, degree=3):
        ipl_t = np.linspace(0.0, len(x) - 1, len(x))
        spl_i_x = scipy_interpolate.make_interp_spline(ipl_t, x, k=degree)
        spl_i_y = scipy_interpolate.make_interp_spline(ipl_t, y, k=degree)
        travel = np.linspace(0.0, len(x) - 1, n_path_points)
        return spl_i_x(travel), spl_i_y(travel)

    def interpolate_park_path(self, path):
        choices = np.arange(0,len(path),2)
        if len(path)-1 not in choices:
            choices = np.append(choices,len(path)-1)
        way_point_x = path[choices,0]
        way_point_y = path[choices,1]
        n_course_point = 50
        rix, riy = self.interpolate_b_spline_path(way_point_x, way_point_y, n_course_point)
        new_path = (np.vstack([rix,riy]).T)
        # new_path[new_path<0] = 0
        return new_path


    def plan_park_up_right(self, x1, y1):       
            s = 4
            l = 8
            d = 2
            w = 4

            x0 = x1 + d + w
            y0 = y1 + l + s
            
            curve_x = np.array([])
            curve_y = np.array([])
            y = np.arange(y1,y0+1)
            circle_fun = (6.9**2 - (y-y0)**2)
            x = (np.sqrt(circle_fun[circle_fun>=0]) + x0-6.9)
            y = y[circle_fun>=0]
            choices = x>x0-6.9/2
            x=x[choices]
            y=y[choices]
            curve_x = np.append(curve_x, x[::-1])
            curve_y = np.append(curve_y, y[::-1])
            
            y = np.arange(y1,y0+1)
            circle_fun = (6.9**2 - (y-y1)**2)
            x = (np.sqrt(circle_fun[circle_fun>=0]) + x1+6.9)
            y = y[circle_fun>=0]
            x = (x - 2*(x-(x1+6.9)))
            choices = x<x1+6.9/2
            x=x[choices]
            y=y[choices]
            curve_x = np.append(curve_x, x[::-1])
            curve_y = np.append(curve_y, y[::-1])

            park_path = np.vstack([curve_x, curve_y]).T
            return park_path

    def plan_park_up_left(self, x1, y1):       
            s = 4
            l = 8
            d = 2
            w = 4

            x0 = x1 - d - w
            y0 = y1 + l + s
            
            curve_x = np.array([])
            curve_y = np.array([])
            y = np.arange(y1,y0+1)
            circle_fun = (6.9**2 - (y-y0)**2)
            x = (np.sqrt(circle_fun[circle_fun>=0]) + x0+6.9)
            y = y[circle_fun>=0]
            x = (x - 2*(x-(x0+6.9)))
            choices = x<x0+6.9/2
            x=x[choices]
            y=y[choices]
            curve_x = np.append(curve_x, x[::-1])
            curve_y = np.append(curve_y, y[::-1])
            
            y = np.arange(y1,y0+1)
            circle_fun = (6.9**2 - (y-y1)**2)
            x = (np.sqrt(circle_fun[circle_fun>=0]) + x1-6.9)
            y = y[circle_fun>=0]
            choices = x>x1-6.9/2
            x=x[choices]
            y=y[choices]
            curve_x = np.append(curve_x, x[::-1])
            curve_y = np.append(curve_y, y[::-1])

            park_path = np.vstack([curve_x, curve_y]).T
            return park_path


    def plan_park_down_right(self, x1,y1):
            s = 4
            l = 8
            d = 2
            w = 4

            x0 = x1 + d + w
            y0 = y1 - l - s
            
            curve_x = np.array([])
            curve_y = np.array([])
            y = np.arange(y0,y1+1)
            circle_fun = (6.9**2 - (y-y0)**2)
            x = (np.sqrt(circle_fun[circle_fun>=0]) + x0-6.9)
            y = y[circle_fun>=0]
            choices = x>x0-6.9/2
            x=x[choices]
            y=y[choices]
            
            curve_x = np.append(curve_x, x)
            curve_y = np.append(curve_y, y)
            
            y = np.arange(y0,y1+1)
            circle_fun = (6.9**2 - (y-y1)**2)
            x = (np.sqrt(circle_fun[circle_fun>=0]) + x1+6.9)
            x = (x - 2*(x-(x1+6.9)))
            y = y[circle_fun>=0]
            choices = x<x1+6.9/2
            x=x[choices]
            y=y[choices]
            curve_x = np.append(curve_x, x)
            curve_y = np.append(curve_y, y)
            
            park_path = np.vstack([curve_x, curve_y]).T
            return park_path


    def plan_park_down_left(self, x1,y1):
            s = 4
            l = 8
            d = 2
            w = 4

            x0 = x1 - d - w
            y0 = y1 - l - s
            
            curve_x = np.array([])
            curve_y = np.array([])
            y = np.arange(y0,y1+1)
            circle_fun = (6.9**2 - (y-y0)**2)
            x = (np.sqrt(circle_fun[circle_fun>=0]) + x0+6.9)
            y = y[circle_fun>=0]
            x = (x - 2*(x-(x0+6.9)))
            choices = x<x0+6.9/2
            x=x[choices]
            y=y[choices]
            curve_x = np.append(curve_x, x)
            curve_y = np.append(curve_y, y)
            
            y = np.arange(y0,y1+1)
            circle_fun = (6.9**2 - (y-y1)**2)
            x = (np.sqrt(circle_fun[circle_fun>=0]) + x1-6.9)
            y = y[circle_fun>=0]
            choices = x>x1-6.9/2
            x=x[choices]
            y=y[choices]
            curve_x = np.append(curve_x, x)
            curve_y = np.append(curve_y, y)
            
            park_path = np.vstack([curve_x, curve_y]).T
            return park_path