import numpy as np
import math
import time
import casadi as ca

from config import *

class Drone:
    def __init__(self, index:int, state:np.array, control:np.array, radius):
        self.index = index
        
        # Drone state and control
        self.time_stamp = 0.0
        self.state = state
        self.control = control

        self.n_state = 6
        self.n_control = 3

        # Drone radius
        self.radius = radius

        # Drone control bounds
        self.control_max = np.array([ 1.5, 1.5, 1.5])
        self.control_min = -self.control_max
        
        # Store drone path
        # self.path = [np.concatenate([[self.time_stamp], self.state, self.control])]
        self.path = []

    def updateState(self, control:np.array, dt:float):
        """
        Computes the states of drone after applying control signals
        """
        
        # Update
        position = self.state[:3]
        velocity = self.state[3:]

        next_position = position + velocity*dt
        next_velocity = velocity + control*dt

        self.state = np.concatenate([next_position, next_velocity])
        self.control = control
        self.time_stamp = self.time_stamp + dt

        # Shift predictive values
        self.states_prediction[:-1,:] = self.states_prediction[1:,:]
        self.controls_prediction[:-1,:] = self.controls_prediction[1:,:]

        # Store
        self.path.append(np.concatenate([[self.time_stamp], self.state, self.control, self.neighbors_distance]))

    def setupController(self, horizon_length=20, dt=0.1):
        # nmpc timestep
        self.nmpc_timestep = dt

        # Predictive length
        self.horizon_length = horizon_length

        self.opti = ca.Opti()
        # control variables, linear velocity and angular velocity
        self.opt_controls = self.opti.variable(self.horizon_length, self.n_control)
        self.opt_states = self.opti.variable(self.horizon_length+1, self.n_state)

        f = lambda x_, u_: ca.horzcat(*[
            x_[3:],
            u_
        ])
  
        # initial condition
        self.opt_start = self.opti.parameter(self.n_state)

        self.opti.subject_to(self.opt_states[0, :] == self.opt_start.T)
        for i in range(self.horizon_length):
            x_next = self.opt_states[i, :] + f(self.opt_states[i, :], self.opt_controls[i, :])*self.nmpc_timestep
            self.opti.subject_to(self.opt_states[i+1, :] == x_next)

        # add constraints to obstacle
        for i in range(self.horizon_length+1):
            for j in range(OBSTACLES.shape[0]):
                temp_constraints_ = ca.sqrt((self.opt_states[i,0]-OBSTACLES[j,0])**2 + \
                                            (self.opt_states[i,1]-OBSTACLES[j,1])**2) - (ROBOT_RADIUS + OBSTACLES[j,2])
                self.opti.subject_to(temp_constraints_ > 0.0)
        
        # add constraints to collision
        self.opt_neighbors = [self.opti.parameter(self.horizon_length+1, self.n_state) for _ in range(NEIGHBOR_NUM)]
        for idx in range(NEIGHBOR_NUM):
            opt_neighbor = self.opt_neighbors[idx]
            for i in range(self.horizon_length+1):
                temp_constraints_ = ca.norm_2(self.opt_states[i,:3]-opt_neighbor[i,:3])
                self.opti.subject_to(temp_constraints_ > 2*self.radius)
        
        self.opti.subject_to(self.opti.bounded(self.control_min[0], self.opt_controls[:,0], self.control_max[0]))
        self.opti.subject_to(self.opti.bounded(self.control_min[1], self.opt_controls[:,1], self.control_max[1]))
        self.opti.subject_to(self.opti.bounded(self.control_min[2], self.opt_controls[:,2], self.control_max[2]))

        opts_setting = {'ipopt.max_iter': 1e8,
                        'ipopt.print_level': 0,
                        'print_time': 0,
                        'ipopt.acceptable_tol': 1e-1,
                        'ipopt.acceptable_obj_change_tol': 1e-1}
        self.opti.solver('ipopt', opts_setting)

        # History predictive horizon
        self.states_prediction = np.ones((horizon_length+1, self.n_state))*self.state
        self.controls_prediction = np.zeros((self.horizon_length, self.n_control))
    
    def computeControlSignal(self, drones):
        """
        Computes control velocity of the copter
        """
        # cost function
        obj = self.costFunction(self.opt_states, self.opt_controls, drones)
        self.opti.minimize(obj)

        # set parameter, here only update initial state of x (x0)
        self.opti.set_value(self.opt_start, self.state)

        # set parameter, here only update predictive state of neighbors
        neighbor_index = self.getNearestNeighbors(drones)
        self.neighbors_distance = np.array([np.linalg.norm(self.state[:3]-drones[i].state[:3]) for i in neighbor_index])
        for i, idx in enumerate(neighbor_index):
            self.opti.set_value(self.opt_neighbors[i], drones[idx].states_prediction)

        # provide the initial guess of the optimization targets
        self.opti.set_initial(self.opt_states, self.states_prediction)
        self.opti.set_initial(self.opt_controls, self.controls_prediction)

        # solve the problem
        sol = self.opti.solve()
        
        ## obtain the control input
        self.controls_prediction = sol.value(self.opt_controls)
        self.states_prediction = sol.value(self.opt_states)
        return self.controls_prediction[0,:]

    def costFunction(self, opt_states, opt_controls, drones):

        c_u = self.costControl(opt_controls)
        c_sep = self.costSeparation(opt_states, drones)
        c_dir = self.costDirection(opt_states)
        c_nav = self.costNavigation(opt_states)
        total = W_sep*c_sep + W_dir*c_dir + W_nav*c_nav + W_u*c_u

        return total

    def costControl(self, u):
        cost_u = 0
        for i in range(self.horizon_length):
            control = u[i,:]
            cost_u += ca.mtimes([control, control.T])
        # print("u: ", cost_u.shape)
        return cost_u

    def costSeparation(self, traj, drones):
        cost_sep = 0
        neighbor_set = self.getNearestNeighbors(drones)
        for j in neighbor_set:
            # if j == self.index:
            #     continue
            for i in range(self.horizon_length+1):
                pos_rel = drones[j].states_prediction[i,:3] - traj[i,:3].T
                cost_sep += (ca.mtimes([pos_rel.T,pos_rel]) - DREF**2)**2
        # print("sep: ", cost_sep.shape)
        return cost_sep

    def costDirection(self, traj):
        cost_dir = 0
        for i in range(self.horizon_length+1):
            vel = traj[i,3:]
            cost_dir += (1 - ca.mtimes(vel,UREF)**2/(ca.mtimes([vel,vel.T])+1e-10))**2
        # print("dir: ", cost_dir.shape)
        return cost_dir
    
    def costNavigation(self, traj):
        cost_nav = 0
        for i in range(self.horizon_length+1):
            vel = traj[i,3:]
            cost_nav += (ca.mtimes([vel,vel.T]) - VREF**2)**2
        # print("nav: ", cost_nav.shape)
        return cost_nav

    def getNearestNeighbors(self, drones):
        distance_list = []
        for i in range(NUM_UAV):
            dis = np.linalg.norm(self.state[:3] - drones[i].state[:3])
            distance_list.append(dis)
        distance_list = np.array(distance_list)
        neighbor_set = np.argsort(distance_list)[1:1+NEIGHBOR_NUM]
        return neighbor_set