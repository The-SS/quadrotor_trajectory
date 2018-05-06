#!/usr/bin/env python
'''
We generate the minimum snap trajectory between m waypoints
Based on:
Minimum Snap Trajectory Generation and Control for Quadrotors
By
Daniel Mellinger and Vijay Kumar
Thru
2011 IEEE International Conference on Robotics and Automation Shanghai International Conference Center
May 9-13, 2011, Shanghai, China

Author:
Sleiman Safaoui
git:
The-SS
Email:
snsafaoui@gmail.com
sleiman.safaoui@utdallas.edu

Date:
April 3, 2018
'''
from cvxpy import *
import numpy as np
import csv
import math
import matplotlib.pyplot as plt

class SnapTrajectory:
    def __init__(self, degree):
        self.degree = degree
        self.variables = []
        self.timestamps = []

    def traj(self, waypoints):
        '''
        generates the trajectory between input Waypoints
        waypoints should be passed in the following format:
        waypoints = [[ x0, dx0, d2x0, d3x0, d4x0 ],
                     [ y0, dy0, d2y0, d3y0, d4y0 ],
                     [ z0, dz0, d2z0, d3z0, d4z0 ],
                     [ psi0, dpsi0, d2psi0 ],
                     [t0],
                     [ x1, dx1, d2x1, d3x1, d4x1 ],
                     [ y1, dy1, d2y1, d3y1, d4y1 ],
                     [ z1, dz1, d2z1, d3z1, d4z1 ],
                     [ psi1, dpsi1, d2psi1 ],
                     [t1],
                     ...,
                     [ x<m>, dx<m>, d2x<m>, d3x<m>, d4x<m> ],
                     [ y<m>, dy<m>, d2y<m>, d3y<m>, d4y<m> ],
                     [ z<m>, dz<m>, d2z<m>, d3z<m>, d4z<m> ],
                     [ psi<m>, dpsi<m>, d2psi<m> ],
                     [t<m>]
                     ]
                     t0 must be set to zero
                     x0, y0, z0, psi0 and their derivatives represent the initial conditions
        '''
        # check if waypoints were inputted correctly
        l = len(waypoints)
        if (l%5 != 0):
            print('Error! Incorrect input.')
            return
        if (l/5 < 2):
            print('no where to go')
            return
        # parse inputs
        # initial x,y,z,psi,t
        xi = np.zeros(5)
        yi = np.zeros(5)
        zi = np.zeros(5)
        psii = np.zeros(3)
        ti = np.zeros(1)
        # final x,y,z,psi,t
        xf = np.zeros(5)
        yf = np.zeros(5)
        zf = np.zeros(5)
        psif = np.zeros(3)
        tf = np.zeros(1)
        for i in range(int(l/5)):
            # update current destination x,y,z,psi,t
            for j in range(len(waypoints[5*i])):
                xf[j]=waypoints[5*i][j]
            for j in range(len(waypoints[5*i+1])):
                yf[j]=waypoints[5*i+1][j]
            for j in range(len(waypoints[5*i+2])):
                zf[j]=waypoints[5*i+2][j]
            for j in range(len(waypoints[5*i+3])):
                psif[j]=waypoints[5*i+3][j]
            tf[0] = waypoints[5*i+4][0]
            if ( tf[0] - ti[0] < 0):
                print('Error! Time not increasing between waypoints.')
                return
            # find the optimization variables
            if i > 0: #skip first waypoint. It is the initial condition
                x_v = np.array(self.min_snap(ti[0], tf[0], xi, xf))
                y_v = np.array(self.min_snap(ti[0], tf[0], yi, yf))
                z_v = np.array(self.min_snap(ti[0], tf[0], zi, zf))
                psi_v = np.array(self.min_acc(ti[0], tf[0], psii, psif))
                self.variables.append(x_v)
                self.variables.append(y_v)
                self.variables.append(z_v)
                self.variables.append(psi_v)
            self.timestamps.append(tf[0])
            xi[:] = xf[:] # store old destination x in current initial
            yi[:] = yf[:] # store old destination y in current initial
            zi[:] = zf[:] # store old destination z in current initial
            psii[:] = psif[:] # store old destination psi in current initial
            ti[0] = tf[0] # store old destination time in current initial
            #print(xf,yf,zf,psif,tf)
        #TODO: store data
        print("Optimization Variables:")
        print(self.variables)
        print("Time:")
        print(self.timestamps)

    def traj_stepwise (self, waypoints):
            '''
            generates the trajectory between input Waypoints
            waypoints should be passed in the following format:
            waypoints = [[ x0, dx0, d2x0, d3x0, d4x0 ],
                         [ y0, dy0, d2y0, d3y0, d4y0 ],
                         [ z0, dz0, d2z0, d3z0, d4z0 ],
                         [ psi0, dpsi0, d2psi0 ],
                         [t0],
                         [ x1, dx1, d2x1, d3x1, d4x1 ],
                         [ y1, dy1, d2y1, d3y1, d4y1 ],
                         [ z1, dz1, d2z1, d3z1, d4z1 ],
                         [ psi1, dpsi1, d2psi1 ],
                         [t1],
                         ...,
                         [ x<m>, dx<m>, d2x<m>, d3x<m>, d4x<m> ],
                         [ y<m>, dy<m>, d2y<m>, d3y<m>, d4y<m> ],
                         [ z<m>, dz<m>, d2z<m>, d3z<m>, d4z<m> ],
                         [ psi<m>, dpsi<m>, d2psi<m> ],
                         [t<m>]
                         ]
                         t0 must be set to zero
                         x0, y0, z0, psi0 and their derivatives represent the initial conditions
            '''
            # check if waypoints were inputted correctly
            l = len(waypoints)
            if (l%5 != 0):
                print('Error! Incorrect input.')
                return
            if (l/5 < 2):
                print('no where to go')
                return
            # parse inputs
            # initial x,y,z,psi,t
            xi = np.zeros(5)
            yi = np.zeros(5)
            zi = np.zeros(5)
            psii = np.zeros(3)
            ti = np.zeros(1)
            # final x,y,z,psi,t
            xf = np.zeros(5)
            yf = np.zeros(5)
            zf = np.zeros(5)
            psif = np.zeros(3)
            tf = np.zeros(1)
            for i in range(int(l/5)):
                # update current destination x,y,z,psi,t
                for j in range(len(waypoints[5*i])):
                    xf[j]=waypoints[5*i][j]
                for j in range(len(waypoints[5*i+1])):
                    yf[j]=waypoints[5*i+1][j]
                for j in range(len(waypoints[5*i+2])):
                    zf[j]=waypoints[5*i+2][j]
                for j in range(len(waypoints[5*i+3])):
                    psif[j]=waypoints[5*i+3][j]
                tf[0] = waypoints[5*i+4][0]
                if ( tf[0] - ti[0] < 0):
                    print('Error! Time not increasing between waypoints.')
                    return
                # find the optimization variables
                if i > 0: #skip first waypoint. It is the initial condition
                    x_v = np.array(self.min_snap(0.0, tf[0]-ti[0], xi, xf))
                    y_v = np.array(self.min_snap(0.0, tf[0]-ti[0], yi, yf))
                    z_v = np.array(self.min_snap(0.0, tf[0]-ti[0], zi, zf))
                    psi_v = np.array(self.min_acc(0.0, tf[0]-ti[0], psii, psif))
                    self.variables.append(x_v)
                    self.variables.append(y_v)
                    self.variables.append(z_v)
                    self.variables.append(psi_v)
                self.timestamps.append(tf[0])
                xi[:] = xf[:] # store old destination x in current initial
                yi[:] = yf[:] # store old destination y in current initial
                zi[:] = zf[:] # store old destination z in current initial
                psii[:] = psif[:] # store old destination psi in current initial
                ti[0] = tf[0] # store old destination time in current initial
                #print(xf,yf,zf,psif,tf)
            #TODO: store data
            print("Optimization Variables:")
            print(self.variables)
            print("Time:")
            print(self.timestamps)

    def min_snap(self, ti, tf, wi, wf):
        '''
        generates minimum snap trajectories (used for position)
        arguments:
            ti: initial time /at start waypoint
            tf: final time /at end waypoint (tf-ti = duration of trajectory between the two points)
            w: Waypoint provided in the following format:
               [ v0, dv0, d2v0, ... ]
               nothing can be omitted
        '''
        # generating coefficients of variables obtained from differentiating the function
        C = np.zeros((self.degree+1,self.degree+1))
        for i in range(self.degree+1):
            if i < 4:
                pass
            else:
                C[i,i] = math.factorial(i)/math.factorial(i-4)
        # generating matrix of integral of time variables
        T_mat = np.zeros((self.degree+1,self.degree+1), dtype=float)
        for i in range(4,self.degree+1):
            for j in range(4,self.degree+1):
                T_mat[i,j] = 1.0/((i-4)+(j-4)+1) * (tf - ti)**((i-4)+(j-4)+1)
        # time vectors
        t_i = np.zeros(self.degree+1) #t poly at start waypoint
        t_f = np.zeros(self.degree+1) #t poly at end waypoint
        for i in range(self.degree+1):
            t_i[i] = ti**i
            t_f[i] = tf**i
        # dt1 vectors (1st derivative)
        dt1_i = np.zeros(self.degree+1) #dt1 poly at start waypoint
        dt1_f = np.zeros(self.degree+1) #dt1 poly at start waypoint
        for i in range(1,self.degree+1):
            dt1_i[i] = i*ti**(i-1)
            dt1_f[i] = i*tf**(i-1)
        # dt2 vectors (2nd derivative)
        dt2_i = np.zeros(self.degree+1) #dt2 poly at start waypoint
        dt2_f = np.zeros(self.degree+1) #dt2 poly at start waypoint
        for i in range(2,self.degree+1):
            dt2_i[i] = i*(i-1)*ti**(i-2)
            dt2_f[i] = i*(i-1)*tf**(i-2)
        # dt3 vectors (3rd derivative)
        dt3_i = np.zeros(self.degree+1) #dt3 poly at start waypoint
        dt3_f = np.zeros(self.degree+1) #dt3 poly at start waypoint
        for i in range(3,self.degree+1):
            dt3_i[i] = i*(i-1)*(i-2)*ti**(i-3)
            dt3_f[i] = i*(i-1)*(i-2)*tf**(i-3)
        # dt4 vectors (4th derivative)
        dt4_i = np.zeros(self.degree+1) #dt4 poly at start waypoint
        dt4_f = np.zeros(self.degree+1) #dt4 poly at start waypoint
        for i in range(4,self.degree+1):
            dt4_i[i] = i*(i-1)*(i-2)*(i-3)*ti**(i-4)
            dt4_f[i] = i*(i-1)*(i-2)*(i-3)*tf**(i-4)
        # finding the matrix H with all the constants: H = C*T_mat*C
        H = np.matmul(C, np.matmul(T_mat, C))
        # print("H",H)
        # optimization
        V = Variable(self.degree+1) # optimization variables
        objective = Minimize(quad_form(V,H)) #objective function see https://github.com/cvxr/CVX/blob/master/functions/quad_form.m
        #constraints = [t_i*V == wi[0], dt1_i*V == wi[1], dt2_i*V == wi[2], dt3_i*V == wi[3], dt4_i*V == wi[4], t_f*V == wf[0], dt1_f*V == wf[1], dt2_f*V == wf[2], dt3_f*V == wf[3], dt4_f*V == wf[4]] # constraints
        constraints = [t_i*V == wi[0], dt1_i*V == wi[1], t_f*V == wf[0], dt1_f*V == wf[1]] # constraints
        prob = Problem(objective, constraints) # optimization problem
        prob.solve(solver='SCS', eps=1e-12) # solving optimization problem

        # analytical solution
        # Q = np.zeros((self.degree+5, self.degree+5))
        # A = np.zeros((4,self.degree+1))
        # A[0,:] = [1., 0., 0., 0., 0., 0., 0., 0., 0.]
        # A[1,:] = [0., 1., 0., 0., 0., 0., 0., 0., 0.]
        # A[2,:] = [1., tf, tf**2., tf**3., tf**4., tf**5., tf**6., tf**7., tf**8.]
        # A[3,:] = [0., tf, 2.*tf**1., 3.*tf**2., 4.*tf**3., 5.*tf**4., 6.*tf**5., 7.*tf**6., 8.*tf**7.]
        # Q[0:self.degree+1, 0:self.degree+1] = 2*H
        # Q[self.degree+1:self.degree+5, 0:self.degree+1] = A[:,:]
        # Q[0:self.degree+1,self.degree+1:self.degree+5] = A.T
        # B = np.zeros((13,1))
        # B[9] = wi[0]
        # B[10] = wi[1]
        # B[11] = wf[0]
        # B[12] = wf[1]
        # X = np.linalg.inv(Q)*B
        # print('X',X)

        return np.array(V.value)[:,0]

    def min_acc(self, ti, tf, wi, wf):
        '''
        generates minimum acceleration trajectory (used for yaw)
        '''
        C = np.zeros((self.degree+1,self.degree+1))
        for i in range(self.degree+1):
            if i < 2:
                pass
            else:
                C[i,i] = math.factorial(i)/math.factorial(i-2)
        # generating matrix of integral of time variables
        T_mat = np.zeros((self.degree+1,self.degree+1), dtype=float)
        for i in range(2,self.degree+1):
            for j in range(2,self.degree+1):
                T_mat[i,j] = 1.0/((i-2)+(j-2)+1) * (tf - ti)**((i-2)+(j-2)+1)
        # matrix for powers of tf
        t_i = np.zeros(self.degree+1)
        t_f = np.zeros(self.degree+1)
        for i in range(self.degree+1):
            t_i[i] = ti**i
            t_f[i] = tf**i
        # matrix for powers of derivative of tf
        dt1_i = np.zeros(self.degree+1)
        dt1_f = np.zeros(self.degree+1)
        for i in range(1,self.degree+1):
            dt1_i[i] = i*ti**(i-1)
            dt1_f[i] = i*tf**(i-1)
        # finding the matrix H with all the constants: H = C.T_mat.C
        H = np.matmul(C, np.matmul(T_mat, C))
        # defining optimization vairables
        V = Variable(self.degree+1)
        # setting up the optimization problem
        objective = Minimize(quad_form(V,H)) #see https://github.com/cvxr/CVX/blob/master/functions/quad_form.m
        constraints = [t_i*V == wi[0], dt1_i*V == wi[1], t_f*V == wf[0], dt1_f*V == wf[1]] # constraints defined by waypoints
        prob = Problem(objective, constraints)
        #solving the optimization problem
        prob.solve(solver='SCS', eps=1e-12)
        # return results
        return np.array(V.value)[:,0]

    def plot_traj(self):
        '''
        plots graphs for x, y, z, and psi
        '''
        for j in range(4):
            t_all = []
            traj_all = []
            for i in range(len(self.timestamps)-1):
                ti = self.timestamps[i]
                tf = self.timestamps[i+1]
                sigma = self.variables[4*i+j]
                if i == len(self.timestamps)-2:
                    t = np.linspace(ti, tf, num=100, endpoint=True)
                else:
                    t = np.linspace(ti, tf, num=100, endpoint=False)
                traj = np.zeros(len(t))
                for k in range(len(t)):
                    t_poly = []
                    for d in range(self.degree+1):
                        t_poly.append(t[k]**d)
                    traj[k] = np.dot(sigma, t_poly)
                t_all.extend(t)
                traj_all.extend(traj)
            plt.figure(1)
            if j == 0:
                plt.subplot(221)
                plt.title('x-pos')
                plt.plot(t_all, traj_all)
            if j == 1:
                plt.subplot(222)
                plt.title('y-pos')
                plt.plot(t_all, traj_all)
            if j == 2:
                plt.subplot(223)
                plt.title('z-pos')
                plt.plot(t_all, traj_all)
            if j == 3:
                plt.subplot(224)
                plt.title('psi')
                plt.plot(t_all, traj_all)
        plt.show()

    def output_csv(self):
        #TODO: output variables to csv file
        file = open('cf_traj.csv', "wb")
        wr = csv.writer(file, delimiter=',')
        for i in range(len(self.variables)/4):
            # print(len(self.variables)/4)
            coef = []
            coef.append(self.timestamps[i+1]-self.timestamps[i])
            for j in range(4):
                # print('idx',4*i+j)
                coef.extend(self.variables[4*i+j].tolist())
            # print('coef',coef)
            wr.writerow(coef)
        file.close()

    def input_csv(self, file_name):
        data = np.loadtxt(file_name, delimiter=",", usecols=range(33))
        data_time = []
        data_coef =[]
        for i in range(len(data)):
            print(data[i])
            print('00000',data[i][0])
            print('cooooef', data[i][1:len(data[i])-1])
            data_time.append(data[i][0])
            data_coef.append(data[i][1:len(data[i])-1].tolist())
            data_coef[i].extend([0.0])
        self.timestamps = data_time
        self.variables = data_coef
        # data = np.loadtxt(file_name, delimiter=",")
        return (data_time, data_coef)

    # def input_csv(self, file_name):
    #     data = load_csv(file_name)




if __name__ == "__main__":
    st = SnapTrajectory(7)
    # w = [[0],[0],[0],[0],[0],  [1],[1],[1],[0],[4], [2],[2],[0],[0],[8]]
    # w = [[2],[2],[3],[0],[0], [1],[0],[0],[0],[2]]
    # w = [[0,0],[0,0],[0,0],[0,0],[0], [-1,0.5],[1,-0.5],[1,0.5],[0,0],[4], [0.5,0],[0.5,0],[0.5,0],[0,0],[8]]
    # w = [[0,0],[0,0],[0,0],[0,0],[0], [1,0],[1,0],[1,0],[0,0],[1]]
    # w = [[1,0],[1,0],[1,0],[0,0],[0], [2,0],[2,0],[2,0],[0,0],[1]]
    # w = [[1,0],[1,0],[1,0],[0,0],[0], [0.5,0],[0.5,0],[0.5,0],[0,0],[1]]
    # w = [[3],[2],[0],[0],[0],  [5],[5],[5],[5],[3]]
    # w = [[0,0],[0,0],[0,0],[0,0],[0], [1,0],[1,0],[1,0],[1,0],[4], [0.,0.],[0.,0.],[0.,0.],[0.,0.],[8]]
    #Circle
    w = [[0,0],[0,0],[0],[0],[0]]
    #2
    w.append([0.5,0])
    w.append([-0.5,0])
    w.append([0.2])
    w.append([0])
    w.append([2])
    #3
    w.append([-.5,0])
    w.append([-0.5,0])
    w.append([0.4])
    w.append([0])
    w.append([4])
    #4
    w.append([0,0])
    w.append([-0.25,0])
    w.append([0.5])
    w.append([0])
    w.append([6])



    # st.traj(w)
    st.traj_stepwise(w)
    st.plot_traj()
    st.output_csv()
    # data = st.input_csv('traj_utd.csv')
    # print('data_time',data[0])
    # time = data[0]
    # print('data_var', data[1])
    # var = data[1]
    # st.plot_traj()
    #


    # # rebuilding graph from csv file
    # ts = 0;
    # t_all = []
    # traj_all_x = []
    # traj_all_y = []
    # traj_all_z = []
    # traj_all_psi = []
    # for i in range(len(time)):
    #     t = np.linspace(ts, ts+time[i], num = 100, endpoint = False)
    #     print('t0',ts)
    #     ts += time[i]
    #     print('t1',ts)
    #     x = var[i][0:8]
    #     print('x',x)
    #     y = var[i][8:16]
    #     print('y',y)
    #     z = var[i][16:24]
    #     print('z',z)
    #     psi = var[i][24:32]
    #     print('psi',psi)
    #
    #     traj_x = np.zeros(len(t))
    #     traj_y = np.zeros(len(t))
    #     traj_z = np.zeros(len(t))
    #     traj_psi = np.zeros(len(t))
    #
    #     for k in range(len(t)):
    #         t_poly = []
    #         for d in range(7+1):
    #             t_poly.append(t[k]**d)
    #         traj_x[k] = np.dot(x, t_poly)
    #         traj_y[k] = np.dot(y, t_poly)
    #         traj_z[k] = np.dot(z, t_poly)
    #         traj_psi[k] = np.dot(psi, t_poly)
    #     t_all.extend(t)
    #     traj_all_x.extend(traj_x)
    #     traj_all_y.extend(traj_y)
    #     traj_all_z.extend(traj_z)
    #     traj_all_psi.extend(traj_psi)
    #
    # plt.figure(1)
    # plt.title('Dr.Summers')
    # plt.subplot(221)
    # plt.title('x-pos')
    # plt.plot(t_all, traj_all_x)
    # plt.subplot(222)
    # plt.title('y-pos')
    # plt.plot(t_all, traj_all_y)
    # plt.subplot(223)
    # plt.title('z-pos')
    # plt.plot(t_all, traj_all_z)
    # plt.subplot(224)
    # plt.title('psi')
    # plt.plot(t_all, traj_all_psi)
    # plt.show()
