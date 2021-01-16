import numpy as np
from scipy.optimize import minimize


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
    def __init__(self, horiz):
        self.horiz = horiz
        self.R = np.diag([0.01, 0.01])                 # input cost matrix
        self.Rd = np.diag([0.01, 1.0])                 # input difference cost matrix
        self.Q = np.diag([1.0, 1.0, 0.5, 0.5])         # state cost matrix
        self.Qf = self.Q                               # state final matrix

    def mpc_cost(self, u_k, my_car, x_des, y_des, v_des, psi_des):
        dt = my_car.dt            # sampling time
        L = my_car.L              # wehicle length
        x = my_car.x  
        y = my_car.y  
        v = my_car.v  
        psi = my_car.psi 
        u_k = u_k.reshape(self.horiz,2).T
        z_k = np.zeros((4,self.horiz+1))
        cost = 0.0
        desired_state = np.array([x_des,y_des,v_des,psi_des])

        for i in range(self.horiz):
            x_dot = v*np.cos(psi)
            y_dot = v*np.sin(psi)
            v_dot = u_k[0,i]
            psi_dot = v*np.tan(u_k[1,i])/L
            
            x += dt*x_dot
            y += dt*y_dot
            v += dt*v_dot
            psi += dt*psi_dot
            
            z_k[:,i] = [x,y,v,psi]
            cost += np.sum(self.R@(u_k[:,i]**2))
            cost += np.sum(self.Q@((desired_state-z_k[:,i])**2))
            if i < (self.horiz-1):     
                cost += np.sum(self.Rd@((u_k[:,i+1] - u_k[:,i])**2))
        return cost

    def optimize(self, my_car, x_des, y_des, v_des, psi_des):
        bnd = [(-5, 5),(np.deg2rad(-60), np.deg2rad(60))]*self.horiz
        result = minimize(self.mpc_cost, args=(my_car, x_des, y_des, v_des, psi_des), x0 = np.zeros((2*self.horiz)), method='SLSQP', bounds = bnd)
        # accelerate = result.x[0]
        # delta = result.x[1]
        # max_delta = 45
        # if np.deg2rad(-max_delta)<=result.x[1]<=np.deg2rad(max_delta):
        #     delta = result.x[1]
        # elif result.x[1]>np.deg2rad(max_delta):
        #     delta = np.deg2rad(max_delta)
        # else:
        #     delta = np.deg2rad(-max_delta)
        return np.clip(result.x[0], -5,5),  np.clip(result.x[1], np.deg2rad(-45),np.deg2rad(45))



######################################################################################################################################################################

    # def make_model(self, v, phi, delta):        
    #     # matrices
    #     # 4*4
    #     A = np.array([[1, 0, self.dt*np.cos(phi)         , -self.dt*v*np.sin(phi)],
    #                   [0, 1, self.dt*np.sin(phi)         , self.dt*v*np.cos(phi) ],
    #                   [0, 0, 1                           , 0                     ],
    #                   [0, 0, self.dt*np.tan(delta)/self.L, 1                     ]])
    #     # 4*2 
    #     B = np.array([[0      , 0                                  ],
    #                   [0      , 0                                  ],
    #                   [self.dt, 0                                  ],
    #                   [0      , self.dt*v/(self.L*np.cos(delta)**2)]])

    #     # 4*1
    #     C = np.array([[self.dt*v* np.sin(phi)*phi                ],
    #                   [-self.dt*v*np.cos(phi)*phi                ],
    #                   [0                                         ],
    #                   [-self.dt*v*delta/(self.L*np.cos(delta)**2)]])
        
    #     return A, B, C

    # def move(self, accelerate, steer):
    #     delta = np.deg2rad(steer)
    #     u_k = np.array([[accelerate, delta]]).T
    #     A,B,C = self.make_model(self.v, self.phi, delta)
    #     z_k1 = A@self.z_k + B@u_k + C
    #     return u_k, z_k1