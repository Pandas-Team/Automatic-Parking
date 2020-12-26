import numpy as np

class Car_Dynamics:
    def __init__(self, x_0, y_0, v_0, phi_0, sample_time, length):
        self.dt  = sample_time    # sampling time
        self.L   = length          # wehicle length
        self.x = x_0
        self.y = y_0
        self.v = v_0
        self.phi = np.deg2rad(phi_0)

    def make_model(self, v, phi, delta):        
        # matrices
        # 4*4
        A = np.array([[1, 0, self.dt*np.cos(phi)         , -self.dt*v*np.sin(phi)],
                      [0, 1, self.dt*np.sin(phi)         , self.dt*v*np.cos(phi) ],
                      [0, 0, 1                           , 0                     ],
                      [0, 0, self.dt*np.tan(delta)/self.L, 1                     ]])
        # 4*2 
        B = np.array([[0      , 0                                  ],
                      [0      , 0                                  ],
                      [self.dt, 0                                  ],
                      [0      , self.dt*v/(self.L*np.cos(delta)**2)]])

        # 4*1
        C = np.array([[self.dt*v* np.sin(phi)*phi                ],
                      [-self.dt*v*np.cos(phi)*phi                ],
                      [0                                         ],
                      [-self.dt*v*delta/(self.L*np.cos(delta)**2)]])
        
        return A, B, C

    def move(self, accelerate, steer):
        delta = np.deg2rad(steer)
        u_k = np.array([[accelerate, delta]]).T
        z_k = np.array([[self.x, self.y, self.v, self.phi]]).T

        A,B,C = self.make_model(self.v, self.phi, delta)
        
        z_k1 = A@z_k + B@u_k + C
        return z_k1[0,0], z_k1[1,0], z_k1[2,0], z_k1[3,0], 


    def update_state(self, x, y, v, phi):
        self.x = x
        self.y = y
        self.v = v
        self.phi = np.deg2rad(phi)

        
# class MPC_Controller:
#     def __init__():
