import numpy as np
import math
from scipy.optimize import minimize

class Car_Dynamics:
    def __init__(self, x_0, y_0, psi_0, u_0, v_0, r_0, length, dt, Gama):
        self.dt = dt             # sampling time
        self.L = length          # vehicle length
        self.Gama = Gama         # Slope Angle of Earth
        self.x = x_0
        self.y = y_0
        self.psi = psi_0
        self.u = u_0
        self.v = v_0
        self.r = r_0
        self.state = np.array([[self.x, self.y, self.psi, self.u, self.v, self.r]]).T
        self.Parameters = self.generate_parameters()

    def move(self, Theta, Delta):
        x_g0 = self.x
        y_g0 = self.y
        psi0 = self.psi
        u0   = self.u
        v0   = self.v
        r0   = self.r
        Parameters =self.generate_parameters()

        W_wheel0  = u0 / Parameters['R_r']
        W_engine0 = (Parameters['Tau_c']*Parameters['Tau_d'])*W_wheel0

        # Longitudal Dynamic
        T_engine = Parameters['T_e_max']*Theta + Parameters['T_e_min']*(1-Theta) # Engine Torque
        T_aero = 0.5*Parameters['rho_air']*Parameters['S']*Parameters['C_x']*(((Parameters['Eta_c']*Parameters['Eta_d'])/(Parameters['Tau_c']*Parameters['Tau_d']))**3)*(Parameters['R_r']**3)*W_engine0**2 # Equivalent Torque of Aerodynamic Force
        F_r = Parameters['F0'] + Parameters['K']*u0**2
        if u0 == 0:
            F_r =0

        delta_x = F_r*Parameters['R_r']
        T_rolling = Parameters['m_v']*Parameters['g']*math.cos(self.Gama)*F_r*((Parameters['Eta_c']*Parameters['Eta_d'])/(Parameters['Tau_c']*Parameters['Tau_d']))*Parameters['R_r']           # Equivalent Torque of Rolling Resistance Force
        T_slope   = Parameters['m_v']*Parameters['g']*math.sin(self.Gama)*Parameters['R_r']*((Parameters['Eta_c']*Parameters['Eta_d'])/(Parameters['Tau_c']*Parameters['Tau_d']));                                    # Equivalent Torque of Slope Weight Force
        T_load = T_aero + T_rolling + T_slope;
        if T_engine <= T_load:
            T_engine = T_load


        # Runge Kuta 4 can be implemented!!
        W_engine1 = ((T_engine-T_load)/Parameters['I_eq'])*self.dt + W_engine0
        W_wheel1 = (1/(Parameters['Tau_c']*Parameters['Tau_d']))*W_engine1

        u1 = Parameters['R_r']*W_wheel1
        u_dot = (u1 - u0)/self.dt
        W_wheel_dot = (W_wheel1-W_wheel0)/self.dt

        # Lateral Dynamic
        Beta = math.atan(np.array(v0)/np.array(u1))  # [rad] Slide Slip Angle
        if np.isnan(Beta):
            Beta = 0

        alfa_f = Delta - math.atan(np.array(v0 + r0*(Parameters['l1']))/np.array(u1))
        if np.isnan(alfa_f):
            alfa_f = 0

        alfa_r = -math.atan(np.array(v0 - r0*(Parameters['l2']))/ np.array(u1))
        if np.isnan(alfa_r):
            alfa_r = 0

        F_y_1 = Parameters['C1'] * alfa_f
        F_y_2 = Parameters['C2'] * alfa_r
        F_xa = 0.5*Parameters['rho_air']*Parameters['S']*Parameters['C_x']*u1**2
        F_z_2 = (Parameters['m_v']*(u_dot - v0*r0)*Parameters['h1']                 +\
                2*Parameters['I_each_wheel']*(W_wheel_dot)                          +\
                2*Parameters['I_each_wheel']*(W_wheel_dot)*math.cos(Delta)          +\
                F_xa*Parameters['h2']                                               +\
                Parameters['m_v']*Parameters['g']*math.sin(self.Gama)*Parameters['h1']   +\
                Parameters['m_v']*Parameters['g']*math.cos(self.Gama)*(Parameters['l1'] + delta_x))/(2*Parameters['l'])

        F_z_1 = (Parameters['m_v'] * Parameters['g']*math.cos(self.Gama) - 2*F_z_2)/2
        F_x_1 = -(Parameters['I_each_wheel']*W_wheel_dot + F_z_1*delta_x)/Parameters['R_r']

        v_dot = -u1*r0 + (1/Parameters['m_v']) * (2*F_x_1*math.sin(Delta) + 2*F_y_1*math.cos(Delta) + 2*F_y_2-F_xa*math.sin(Beta))
        r_dot = (1/Parameters['j_z'])*((Parameters['l1'] + delta_x)*(2*F_x_1*math.sin(Delta) + 2*F_y_1*math.cos(Delta)) - (Parameters['l2'] - delta_x)*2*F_y_2)

        v1 = v_dot * self.dt + v0
        r1 = r_dot * self.dt + r0
        psi1 = r1*self.dt + psi0
        x_g1 = (u1*math.cos(psi1) - v1*math.sin(psi1))*self.dt + x_g0
        y_g1 = (u1*math.sin(psi1) + v1*math.cos(psi1))*self.dt + y_g0
        # return np.array([[x_g1, y_g1, psi1, u1, v1, r1]]).T

        x_dot = (u1*math.cos(psi1) - v1*math.sin(psi1))
        y_dot = (u1*math.sin(psi1) + v1*math.cos(psi1))
        psi_dot = r1
        u_dot = u_dot
        v_dot = v_dot
        r_dot = r_dot


        return np.array([[x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot]]).T

    def update_state(self, state_dot):
        self.state = self.state + self.dt*state_dot
        self.x = self.state[0,0]
        self.y = self.state[1,0]
        self.psi = self.state[2,0]
        self.u = self.state[3,0]
        self.v = self.state[4,0]
        self.r = self.state[5,0]

    def generate_parameters(self):
        # Parameters
        # Vehicle Model: Renault Mégane Coupé 16V 150 HP 
        Parameters ={}

        # General data
        Parameters['g'] = 9.8

        # Gas data
        r0   = 8315.4                       # [J kgmole/kg/K] Universal gas Constant
        mw   = 28.97                        # [kg/kgmole] Air Molecular Weight
        rair = r0/mw
        pcr  = 0.528                        # [/] Critical Pressure ratio
        rk   = 1.4                          # [/] Specific Heats ratio
        
        # Ambient data
        pair  = 101300                       # [Pa] Atmospheric Pressure
        tair  = 300                          # [K] Ambient Temperature
        umid  = 50                           # [%] Relative humidity
        Parameters['rho_air'] = pair/rair/tair  # [Kg/m3] Air Density
        
        # Transmission data
        Parameters['Eta_d'] = 0.8
        Parameters['Tau_d'] = 3.8
        Parameters['Eta_c'] = 0.8
        Parameters['Tau_c'] = 3.72
        Parameters['T_e_max'] = 223.95
        Parameters['T_e_min'] = 0;-55.98

        # Geometrical data
        Parameters['l']   = 2.468                         # [m] Wheelbase Vehicle
        Parameters['l1']  = 0.9552                        # [m] semi-Wheelbase front Vehicle
        Parameters['l2']  = Parameters['l'] - Parameters['l1']  # [m] semi-Wheelbase rear Vehicle
        Parameters['j_z'] = 1623.8                        # [kgm^2] Inertia around z axle

        # Tyre Data
        Parameters['C1']  = 8405.5              # [Ns/rad] Cornering Stiffness of fornt tyre
        Parameters['C2']  = 8734.2              # [Ns/rad] Cornering Stiffness of rear tyre
        Parameters['R_r'] = 0.3                 # [m] Rolling Effective Radius
        Parameters['F_r'] = 0.9                 # [/] Coefficient of road adhesion
        Parameters['h1']  = 0.45                # [m] Legth between centre of gravity and ground
        Parameters['h2']  = Parameters['h1']       # [m] Legth between point of application Faero and ground
        Parameters['F0']  = 0.9                 # [/] Coefficient of road adhesion
        Parameters['K']   = 0.1                 # [/] Coefficient of road adhesion

        # Rigid driveline
        m_c      = 2.8                        # [kg] Crank Mass
        m_cr     = 0.3                        # [kg] Connecting Rod Big End Mass
        R_c      = 0.03                       # [m] Rc: Crank Radius
        n_cyl    = 4                          # [/] Cylinder Number
        I_fw     = 0.3                        # [kgm^2] Flywheel Inertia
        I_cgi    = ((m_c+m_cr))*R_c**2*n_cyl   # [kgm^2] Crank Gear Inertia
        I_engine = I_cgi + I_fw               # [kgm^2] Total Engine Inertia

        Parameters['I_each_wheel'] = 0.08        # [kgm^2] Wheel Inertia
        Parameters['I_wheel']      = Parameters['I_each_wheel'] * ((Parameters['Eta_c'] * Parameters['Eta_d'])/(Parameters['Tau_c'] * Parameters['Tau_d']))**2   # [kgm^2] Equivalent Wheel Inertia

        mv_sprung         = 1202                                               # [kg] Sprung Mass of Vehicle
        mv_unsprung_front = 105                                                # [kg] Unsprung front Mass of Vehicle
        mv_unsprung_rear  = 55                                                 # [kg] Unsprung rear Mass of Vehicle
        Parameters['m_v']    = mv_sprung + mv_unsprung_front + mv_unsprung_rear   # [kg] Total Mass of Vehicle
        I_chassis         = (Parameters['m_v'] * Parameters['R_r']**2)*((Parameters['Eta_c'] * Parameters['Eta_d'])/(Parameters['Tau_c'] * Parameters['Tau_d']))**2   # [kgm^2] Equivalent Vehicle Inertia
        
        Parameters['I_eq'] = I_engine + I_chassis + 4 * Parameters['I_wheel'] 

        # Aereodynamics data
        Parameters['C_x'] = 0.328                                  # [/] Drag Coefficient
        Parameters['S'] = 1.6 + 0.00056 * (Parameters['m_v']-765)  # [m^2] Frontal Area after J.Y.Wong "Theory of Groung Vehicle"
        return Parameters



    
class MPC_Controller:
    def __init__(self):
        self.horiz = None
        self.R = np.diag([0.01, 0.01])                 # input cost matrix
        self.Rd = np.diag([0.01, 1.0])                 # input difference cost matrix
        self.Q = np.diag([1.0, 1.0])         # state cost matrix
        self.Qf = self.Q       

    def mpc_cost(self, u_k, my_car, points):
        test_car = Car_Dynamics(np.array(my_car.x).copy(), np.array(my_car.y).copy(), np.array(my_car.psi).copy(), np.array(my_car.u).copy(), np.array(my_car.v).copy(), np.array(my_car.r).copy(), length=4, dt=np.array(my_car.dt), Gama=0)

        dt = test_car.dt
        u_k = u_k.reshape(self.horiz,2).T
        z_k = np.zeros((6,self.horiz+1))
        cost = 0.0
        desired_state = points.T
        for i in range(self.horiz):
            state_dot = test_car.move(u_k[0,i],u_k[1,i])
            z_k[:,i] = (test_car.state + dt*state_dot).reshape(6)
            # self.x = self.state[0,0]
            # self.y = self.state[1,0]
            # self.psi = self.state[2,0]
            # self.u = self.state[3,0]
            # self.v = self.state[4,0]
            # self.r = self.state[5,0]

            # x = state[0,0]
            # y = state[1,0]
            # psi = state[2,0]
            # z_k[:,i] = [x,y,psi]
            cost += np.sum(self.R@(u_k[:,i]**2))
            cost += np.sum(self.Q@((desired_state[:,i]-z_k[:2,i])**2))
            if i < (self.horiz-1):     
                cost += np.sum(self.Rd@((u_k[:,i+1] - u_k[:,i])**2))
        return cost

    def optimize(self, my_car, points):
        self.horiz = points.shape[0]
        bnd = [(0.1, 1),(np.deg2rad(-60), np.deg2rad(60))]*self.horiz
        result = minimize(self.mpc_cost, args=(my_car, points), x0 = np.zeros((2*self.horiz)), method='SLSQP', bounds = bnd)
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
