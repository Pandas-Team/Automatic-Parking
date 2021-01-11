import numpy as np
from scipy.optimize import minimize


class Car_Dynamics:
    def __init__(self, x_0, y_0, v_0, phi_0, length, dt):
        self.dt = dt             # sampling time
        self.L = length          # wehicle length
        self.x = x_0
        self.y = y_0
        self.v = v_0
        self.phi = phi_0
        self.state = np.array([[self.x, self.y, self.v, self.phi]]).T

    def move(self, accelerate, delta):
        x_dot = self.v*np.cos(self.phi)
        y_dot = self.v*np.sin(self.phi)
        v_dot = accelerate
        phi_dot = self.v*np.tan(delta)/self.L
        return np.array([[x_dot, y_dot, v_dot, phi_dot]]).T

    def update_state(self, state_dot):
        # self.u_k = command
        # self.z_k = state
        self.state = self.state + self.dt*state_dot
        self.x = self.state[0,0]
        self.y = self.state[1,0]
        self.v = self.state[2,0]
        self.phi = self.state[3,0]

    
class MPC_Controller:
    def __init__(self, horiz):
        self.horiz = horiz
        self.R = np.diag([0.01, 0.01])                 # input cost matrix
        self.Rd = np.diag([0.01, 1.0])                 # input difference cost matrix
        self.Q = np.diag([1.0, 1.0, 0.5, 0.5])         # state cost matrix
        self.Qf = self.Q                               # state final matrix

    def constraint0(self, u_k):
        u_k = u_k.reshape(self.horiz,2).T
        # max_in = np.array([np.array([5]*5),np.repeat(np.deg2rad(30),5)])
        max_in = np.array([[5],[np.deg2rad(30)]])
        return np.sum(np.diag([0.1,10])@(max_in**2-u_k[:,:1]**2))

    def mpc_cost(self, u_k, my_car, x_des, y_des, v_des, phi_des):
        dt = my_car.dt            # sampling time
        L = my_car.L              # wehicle length
        x = my_car.x  
        y = my_car.y  
        v = my_car.v  
        phi = my_car.phi 
        u_k = u_k.reshape(self.horiz,2).T
        z_k = np.zeros((4,self.horiz+1))
        cost = 0.0
        desired_state = np.array([x_des,y_des,v_des,phi_des])

        for i in range(self.horiz):
            x_dot = v*np.cos(phi)
            y_dot = v*np.sin(phi)
            v_dot = u_k[0,i]
            phi_dot = v*np.tan(u_k[1,i])/L
            
            x += dt*x_dot
            y += dt*y_dot
            v += dt*v_dot
            phi += dt*phi_dot
            
            z_k[:,i] = [x,y,v,phi]
            # cost += np.sum(self.R@(u_k[:,i]**2))
            cost += np.sum(self.Q@((desired_state-z_k[:,i])**2))
            if i < (self.horiz-1):     
                cost += np.sum(self.Rd@((u_k[:,i+1] - u_k[:,i])**2))
        return cost

    def optimize(self, my_car, x_des, y_des, v_des, phi_des):
        con0 = {'type': 'ineq', 'fun': self.constraint0}
        result = minimize(self.mpc_cost, args=(my_car, x_des, y_des, v_des, phi_des), x0 = np.zeros((2*self.horiz)), method='SLSQP', constraints=[con0])
        # accelerate = result.x[0]
        # delta = result.x[1]
        # max_delta = 45
        # if np.deg2rad(-max_delta)<=result.x[1]<=np.deg2rad(max_delta):
        #     delta = result.x[1]
        # elif result.x[1]>np.deg2rad(max_delta):
        #     delta = np.deg2rad(max_delta)
        # else:
        #     delta = np.deg2rad(-max_delta)
        return result.x[0],  result.x[1]



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