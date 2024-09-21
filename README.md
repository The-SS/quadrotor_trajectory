# quadrotor_trajectory
This tool implements the minimum snap trajectory optimization problem described in "Minimum Snap Trajectory Generation and Control for Quadrotors" by Daniel Mellinger and Vijay Kumar to generate a polynomial trajectory for quadrotors. 

At a high level, the tool takes in a list of waypoints, sets up and solves the optimization problem, then returns the coefficients of a polynomial in the 3D position and yaw angle `(x,y,z,yaw)`.

The package has been tested on Ubuntu 20.04.3 LTS and macOS Sonoma 14.4.1 (on Apple Silicon) with Python 3.10. See below for instructions on how to install and run the code. 

## Installing the package
The package can be installed in your directory of choice using the following steps.

### Get the package
- Clone the package by navigating to your chosen directory then running the following command in a terminal window:
  ```
  git clone https://github.com/The-SS/quadrotor_trajectory.git
  ```
- Alternatively, you can download the package from [GitHub](https://github.com/The-SS/quadrotor_trajectory), unzip it, and copy it to your chosen directory   

### Install dependencies

#### Using Python Venv
- Install Python 3.10 and Venv
    ```bash
    sudo apt-get install python3.10-full
    sudo apt-get install python3-venv
    ```
- Create a virtual environment
    ```bash
    python3.10 -m venv env
    ```
- Activate the virtual environment
    ```bash
    source env/bin/activate
    ```
- Install the required packages (you must be in the package root directory)
    ```bash
    pip install -r requirements.txt
    ```
  
#### Using Anaconda
- Install Anaconda by following the instructions in the [documentation](https://docs.anaconda.com/anaconda/install/)
- Create a conda environment with Python 3.10 and activate it
    ```bash
    conda create --name env python=3.10
    conda activate env
    ```
  (NOTE: If using PyCharm, you can also setup and activate an environment using its user interface.)
- Install dependencies (you can also use conda to install requirements, but you might have to choose the correct channels)
    ```bash
    pip install -r requirements.txt
    ```

## Using the package
- Edit `min_snap_traj.py` in your favorite text editor
  - Scroll down to main (`if __name__ == "__main__":`)
  - The number 7 in `st = SnapTrajectory(7)` indicated the degree of the polynomial used to generate the trajectory
  - To specify the waypoints, follow the provided example.
  `w = [[0,-0.5],[0,0],[0],[0],[0]]` provides the first waypoint. The other waypoints are appended in the following order: 
    - `[x and its derivatives]`, 
    - then `[y and its derivatives]`, 
    - then `[z and its derivatives]`, 
    - then `[yaw and its derivatives]`, 
    - then `[time at arrival to waypoint]`. 
    - A more detailed description can be seen in the description of the function `traj` or `traj_stepwise`.
  - Use `st.traj(w)` or `st.traj_stepwise(w)` (not both):
    - `st.traj(w)`: generates the trajectories with time intervals `[t0,t1], [t1,t2], ... [t_(n-1), t_n].`
    - `st.traj_stepwise(w)` generates the trajectories with time intervals `[0, t1-t0], [0, t2-t1], ... [0, t_n - t_(n-1)].`. 
    - Note: you can edit the CVXPY solver by changing the `solver` being passed to `prob.solve` in the 
  - To plot the trajectory use `st.plot_traj()`. 
  - If you used `st.traj(w)`, you will get a smooth trajectory. 
  - If you used `st.traj_stepwise(w)`, each segment of the trajectory will be smooth, but there will be a jump/discontinuity at the end of each interval.
  - To output the optimization coefficients to a csv file use `st.output_csv('<file_name>')`. The csv file will appear in `quadrotor_trajectory/scripts`
- Run the `min_snap_traj.py` script:
  ```bash
  cd scripts
  python min_snap_traj.py
  ```
  (Or through your IDE)

