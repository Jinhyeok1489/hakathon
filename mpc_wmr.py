import numpy as np
import math as m
import cvxpy as cp
import matplotlib.pyplot as plt

show_animation = True

# State: x, y, yaw, vx, vy, w
class wmr_state: # Define wmr model state
    def __init__(self, x = 0, y = 0 , yaw = 0):
        self.x = x
        self.y = y
        self.yaw = yaw
        
        self.v = 0
        self.w = 0
    
    def update(self, v = 0, w = 0, dt = 0.01): # Update wmr model with input
        # Save v and w
        self.v = v  # velocity: u(1)
        self.w = w  # angular velocity: u(2)
        # 
        self.yaw = self.yaw + w*dt 
        self.x = self.x +self.v*m.cos(self.yaw)*dt
        self.y = self.y + self.v*m.sin(self.yaw)*dt

        if show_animation: # Update animation by plotting
            # plt.plot(self.x, self.y, '.r')
            plt.arrow(self.x, self.y, np.cos(self.yaw), np.sin(self.yaw), color='r', width=0.01)
            plt.pause(0.0001)
    
    def state(self):
        curr_state = np.array([[self.x], [self.y], [self.yaw]])
        return curr_state
class wmr_states: # Save and append states and inputs
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.w = []
        self.t = []
    
    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.w.append(state.w)

class lmpc_wmr: # Define linearized model predictive control
    def __init__(self, x_ref, u_ref, dt):
        self.t = 0
        self.dt = dt
        self.wmr_ref = wmr_state()
        self.x_ref = x_ref
        self.u_ref = u_ref

        self.x = np.zeros((3,1)) # Reference x
        self.u = np.zeros((2,1)) # Reference u

    def updateModelState(self, v, w, dt, t): 
        self.wmr_ref.update(v, w, dt) 
        self.dt = dt
        self.t = t
    def updateVec(self):
        self.x[0,0] = self.wmr_ref.x
        self.x[1,0] = self.wmr_ref.y
        self.x[2,0] = self.wmr_ref.yaw

        self.u[0,0] = self.wmr_ref.v 
        self.u[1,0] = self.wmr_ref.yaw

    def compute_lmat(self, t): # Compute linearized matrix
        """
        Compute linearized matrix from time 
        """    
        # Compute linearized matrix from reference input
        self.t = t
        height, width = self.x_ref.shape
        len_index = int(width*self.dt)
        index = int(self.t*len_index)
        # print("Index: ", index)

        vr = self.u_ref[0,index]
        thr = self.x_ref[2, index]

        Ar = np.array([[1, 0, -vr*m.sin(thr)*self.dt],[0, 1, vr*m.cos(thr)*self.dt],[0, 0, 1]])
        Br = np.array([[m.cos(thr)*self.dt, 0],[m.sin(thr)*self.dt, 0],[0, self.dt]])

        return Ar, Br

    def multiply_A(self, t, step, u): # Compute pi A
        """
        Compute matrices multiplied A
        Input:
                t:      time
                step:   desired step size
                u:      starting point multiplying
        """
        len_index = int(len(self.x_ref)*self.dt)
        N = step
        Aout = np.eye(3)

        if N == u:
            pass
        else:
            for i in range(N-u):
                Ar, Br = self.compute_lmat(t+len_index*i)
                Aout = Aout@Ar
                # print("Ar: ",Ar)
                # print("Aout: ",Aout)
        return Aout

    def compute_Abar(self, t, step): # Compute Abar by multiplying A
        len_index = int(len(self.x_ref)*self.dt)
        N = step
        Aout = np.eye(3)

        Abar = np.empty((0,3))

        for i in range(N):
            Ar, Br = self.compute_lmat(t+len_index*i)
            Aout = Aout@Ar

            Abar = np.vstack((Abar, Aout))
        return Abar
        
    def compute_Bbar(self, t, step): # Compute Bbar by multiplying A and B
        len_index = int(len(self.x_ref)*self.dt)
        N = step


        Bbar = np.empty((3*N, 0))

        for j in range(N):
            temp1 = np.empty((0,2))
            for i in range(N):
                Ar, Br = self.compute_lmat(t+len_index*j)
                A_temp  = self.multiply_A(t, step, i)
                B_temp = A_temp@Br
                temp1 = np.vstack((temp1, B_temp))
                # print("Ar: ", A_temp)
                # print("Br: ", B_temp)
                # print("Index out: ", j, " Index in : ", i)
                # print("Temp1: ", temp1)
            Bbar = np.hstack((Bbar, temp1))
        return Bbar
    def lmpc_wmr_out(self, x, t, step): # Implement mpc algorithm
        Abar = self.compute_Abar(t, step)
        Bbar = self.compute_Bbar(t, step)
 
        dt = 0.1
        # Initial condition
        v = 0
        w = 0

        n = step*2


        u  = cp.Variable((n,1))

        xbar = Abar@x + Bbar@u
        Q = np.eye(step*3)*1
        Q[2,2] = 0.5
        R = np.eye(step*2)*0.1

        obj = cp.Minimize(cp.quad_form(xbar, Q)+ cp.quad_form(u, R))
        prob = cp.Problem(obj)
        prob.solve()

        output = u.value
        out = output[0:2, 0]
        return out


if __name__ == '__main__':
    wmr1 = wmr_state(0, -1, m.pi/2)

    T = 10.0 # simulation time
    t = 0.0 # time
    wmr_states = wmr_states()
    wmr_states.append(t, wmr1)
    dt = 0.1
    t_mat = np.empty((1,0))

    # Define linear trajectory for trajectory tracking - Example 1
    x_tra = np.linspace(0, 10, int(T/dt))
    y_tra = np.ones(int(T/dt))*1
    th_tra = np.zeros(int(T/dt))
    # Define circular trajectory for trajectory tracking - Example 2

    ref_tra = np.vstack((x_tra, y_tra))
    ref_tra = np.vstack((ref_tra, th_tra))

    ref_v = np.ones(int(T/dt))*np.linalg.norm(ref_tra[0:2, 0]-ref_tra[0:2, 1])/dt
    ref_w = np.ones(int(T/dt))*(ref_tra[2, 1]-ref_tra[2, 0])/dt

    ref_u = np.vstack((ref_v, ref_w))
    ctrl_mpc = lmpc_wmr(ref_tra, ref_u, dt)

    height, width = ref_tra.shape
    len_index = int(width*dt)

    index = 0
    while index < T*len_index-1:

        index = int(t*len_index)
        curr_ref = ref_tra[:, index].reshape(3,1)
        out = ctrl_mpc.lmpc_wmr_out(wmr1.state()-curr_ref, t, 5)
        print(wmr1.state())
        wmr1.update(out[0]+ref_u[0, index], out[1]+ref_u[1, index], dt)
        print("ref_u: ", ref_u[:, index], )
        

        wmr_states.append(t, wmr1)

        t_mat = np.append(t_mat, np.array([t]))
        t = t + dt  # update time
        print("Len_index: ",len_index*T)
        print("Index: ", index)


        if show_animation:
            plt.cla()
            plt.plot(x_tra, y_tra, 'r')
            plt.plot(wmr_states.x, wmr_states.y, 'g')
            plt.grid(True)
            plt.xlabel('x posotion (m)')
            plt.ylabel('y position (m)')
        # plt.show()
        
    # Plot states and Inputs
    fig1 = plt.figure()
    # print("t_mat: ",t_mat)
    # print("t_mat: ", t_mat.shape)
    # print("x_mat: ", (wmr_states.x))
    plt.plot(t_mat[:100], wmr_states.x[0:-2], 'r', linewidth = 2, label = 'x')
    plt.plot(t_mat[:100], wmr_states.y[0:-2], 'b',linewidth = 2, label = 'y')
    plt.plot(t_mat[:100], wmr_states.yaw[0:-2], 'g', linewidth = 2, label = 'yaw')
    plt.plot(t_mat[:100], x_tra, 'r:', markersize = 3, label = 'x_ref')
    plt.plot(t_mat[:100], y_tra, 'b:', markersize = 3, label = 'y_ref')
    plt.plot(t_mat[:100], th_tra, 'g:', markersize = 3, label = 'yaw_ref')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.title('State')
    plt.legend()
    fig2 = plt.figure()
    plt.plot(t_mat[:100], wmr_states.v[0:-2], 'r', linewidth = 2, label = 'v')
    plt.plot(t_mat[:100], wmr_states.w[0:-2], 'b',linewidth = 2, label = 'w')
    plt.plot(t_mat[:100], ref_v, 'r:', markersize = 3, label = 'v_ref')
    plt.plot(t_mat[:100], ref_w, 'b:', markersize = 3, label = 'w_ref')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.title('Input')
    plt.legend()
    plt.show()

