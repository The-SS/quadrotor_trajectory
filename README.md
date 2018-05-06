# quadrotor_trajectory
Generating minimum snap trajectories between waypoints for quadrotors
The package has been tested on MacOS running Sierra 10.12.6

## Installing the package
- Install cvxpy by following the guide at [this link](http://www.cvxpy.org/en/latest/install/index.html).
- clone the package by running the following in terminal:
```
git clone https://github.com/The-SS/quadrotor_trajectory.git
```

## Using the package
- Navigate to the `scripts` folder:
```
cd <path_to_quadrotor_trajectory>/scripts
```
- open `min_snap_traj.py` in your favorite text editor
- Scroll down to main (`if __name__ == "__main__":`)
- The number 7 in `st = SnapTrajectory(7)` indicated the degree of the polynomial used to generate the trajectory
- To specify the waypoints, follow the provided example.
`w = [[0,-0.5],[0,0],[0],[0],[0]]` provides the first waypoint. The other waypoints are appended in the following order: [x and its derivatives], then [y and its derivatives], then [z and its derivatives], then [yaw and its derivatives], then [time at arrival to waypoint]. A more detailed description can be seen in the description of the function `traj` or `traj_stepwise`.
-  Use `st.traj(w)` or `st.traj_stepwise(w)`. `st.traj(w)` will generate the trajectories on intervals [t0,t1], [t1,t2], ... [t_(n-1), t_n]. `st.traj_stepwise(w)` generates the trajectories on intervals [0, t1-t0], [0, t2-t1], ... [0, t_n - t_(n-1)]. Do not use both.
- To plot the trajectory use `st.plot_traj()`. If you used `st.traj(w)`, you will get a smooth trajectory. If you used `st.traj_stepwise(w)`, the trajectory will be discontinues at each waypoint
- To output the optimization coefficients to a csv file use `st.output_csv('<file_name>')`. The csv file will appear in `quadrotor_trajectory/scripts`
