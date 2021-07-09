import numpy as np
from scipy.optimize import minimize
import copy


class Car_Dynamics:
    def __init__(self, x_0, y_0, v_0, psi_0, length, dt):
        self.dt = dt             # sampling time
        self.L = length          # vehicle length
        self.x = x_0
        self.y = y_0
        self.v = v_0
        self.psi = psi_0
        self.state = np.array([[self.x, self.y, self.v, self.psi]]).T

    def move(self, accelerate, delta):
        x_dot = self.v*np.cos(self.psi)
        y_dot = self.v*np.sin(self.psi)
        v_dot = accelerate
        psi_dot = self.v*np.tan(delta)/self.L
        return np.array([[x_dot, y_dot, v_dot, psi_dot]]).T

    def update_state(self, state_dot):
        # self.u_k = command
        # self.z_k = state
        self.state = self.state + self.dt*state_dot
        self.x = self.state[0,0]
        self.y = self.state[1,0]
        self.v = self.state[2,0]
        self.psi = self.state[3,0]

    
class MPC_Controller:
    def __init__(self):
        self.horiz = None
        self.R = np.diag([0.01, 0.01])                 # input cost matrix
        self.Rd = np.diag([0.01, 1.0])                 # input difference cost matrix
        self.Q = np.diag([1.0, 1.0])                   # state cost matrix
        self.Qf = self.Q                               # state final matrix

    def mpc_cost(self, u_k, my_car, points):
        mpc_car = copy.copy(my_car)
        u_k = u_k.reshape(self.horiz, 2).T
        z_k = np.zeros((2, self.horiz+1))
    
        desired_state = points.T
        cost = 0.0

        for i in range(self.horiz):
            state_dot = mpc_car.move(u_k[0,i], u_k[1,i])
            mpc_car.update_state(state_dot)
        
            z_k[:,i] = [mpc_car.x, mpc_car.y]
            cost += np.sum(self.R@(u_k[:,i]**2))
            cost += np.sum(self.Q@((desired_state[:,i]-z_k[:,i])**2))
            if i < (self.horiz-1):     
                cost += np.sum(self.Rd@((u_k[:,i+1] - u_k[:,i])**2))
        return cost

    def optimize(self, my_car, points):
        self.horiz = points.shape[0]
        bnd = [(-5, 5),(np.deg2rad(-60), np.deg2rad(60))]*self.horiz
        result = minimize(self.mpc_cost, args=(my_car, points), x0 = np.zeros((2*self.horiz)), method='SLSQP', bounds = bnd)
        return result.x[0],  result.x[1]



######################################################################################################################################################################

class Linear_MPC_Controller:
    def __init__(self):
        self.horiz = None
        self.R = np.diag([0.01, 0.01])                 # input cost matrix
        self.Rd = np.diag([0.01, 1.0])                 # input difference cost matrix
        self.Q = np.diag([1.0, 1.0])                   # state cost matrix
        self.Qf = self.Q                               # state final matrix
        self.dt=0.2   
        self.L=4                          

    def make_model(self, v, psi, delta):        
        # matrices
        # 4*4
        A = np.array([[1, 0, self.dt*np.cos(psi)         , -self.dt*v*np.sin(psi)],
                    [0, 1, self.dt*np.sin(psi)         , self.dt*v*np.cos(psi) ],
                    [0, 0, 1                           , 0                     ],
                    [0, 0, self.dt*np.tan(delta)/self.L, 1                     ]])
        # 4*2 
        B = np.array([[0      , 0                                  ],
                    [0      , 0                                  ],
                    [self.dt, 0                                  ],
                    [0      , self.dt*v/(self.L*np.cos(delta)**2)]])

        # 4*1
        C = np.array([[self.dt*v* np.sin(psi)*psi                ],
                    [-self.dt*v*np.cos(psi)*psi                ],
                    [0                                         ],
                    [-self.dt*v*delta/(self.L*np.cos(delta)**2)]])
        
        return A, B, C

    def mpc_cost(self, u_k, my_car, points):
        
        u_k = u_k.reshape(self.horiz, 2).T
        z_k = np.zeros((2, self.horiz+1))
        desired_state = points.T
        cost = 0.0
        old_state = np.array([my_car.x, my_car.y, my_car.v, my_car.psi]).reshape(4,1)

        for i in range(self.horiz):
            delta = u_k[1,i]
            A,B,C = self.make_model(my_car.v, my_car.psi, delta)
            new_state = A@old_state + B@u_k + C
        
            z_k[:,i] = [new_state[0,0], new_state[1,0]]
            cost += np.sum(self.R@(u_k[:,i]**2))
            cost += np.sum(self.Q@((desired_state[:,i]-z_k[:,i])**2))
            if i < (self.horiz-1):     
                cost += np.sum(self.Rd@((u_k[:,i+1] - u_k[:,i])**2))
            
            old_state = new_state
        return cost

    def optimize(self, my_car, points):
        self.horiz = points.shape[0]
        bnd = [(-5, 5),(np.deg2rad(-60), np.deg2rad(60))]*self.horiz
        result = minimize(self.mpc_cost, args=(my_car, points), x0 = np.zeros((2*self.horiz)), method='SLSQP', bounds = bnd)
        return result.x[0],  result.x[1]