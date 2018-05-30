# Model Predictive Control
Autonomous systems start with the perception system, which estimates the state of the surrounding environment including landmarks and vehicles and pedestrians. The localization block compares a model to a map to figure out where the system is. The path planning block charts a trajectory using environmental model, the map and vehicle location. Finally, the control loop applies the steering and throttle at every time step to follow this trajectory. Typically, the path planning block passes the reference trajectory to the control block as a polynomial. Third degree polynomials are common so they can fit most roads.
Model predictive control (MPC) is a control theory which was originally proposed for the chemical process industry has been using for the robotics controls in the past decade. MPC uses an optimizer to find the control inputs and minimize the cost function. The basic structure of MPC is shown in the figure below: “A model is used to predict the future plant outputs, based on past and current values and on the proposed optimal future control actions. These actions are calculated by the optimizer taking into account the cost function (where the future tracking error is considered) as well as the constraints.” [1, 2] 
<p align="center">
  <img src="https://github.com/jwangjie/SDC-MPC-Project/blob/master/Figures/Basic%20Structure%20of%20MPC.png" 
       width="500px" height="250px"/>
</p>

For the track driving project specifically, MPC reframes the task of following a trajectory as an optimization problem. The solution to the optimization problem is the optimal trajectory. MPC involves simulating different actuator inputs, predicting that resulting trajectory and selecting that trajectory with the minimum cost. For one step in the process, imagine that we know our current state and the reference trajectory we want to follow, we optimize our actuator inputs at each step in the time horizon in order to minimize the cost of our predicted trajectory. Once we found the lowest cost trajectory, we implement the very first set of actuation commands. Then, we throw away the rest of the trajectory we calculated. Instead of using the old trajectory we predicted, we take our new state and use that to calculate a new optimal trajectory. In that sense, we are constantly calculating inputs over a future horizon. That’s why MPC sometimes is called as Receding Horizon Control. We don’t just carry out the entire trajectory we calculated during the first pass. The reason is that our model is only an approximation that won’t match the real world exactly. Once we perform our actuation commands, our trajectory might not be exactly the same as the trajectory we predicted. Thus, it’s crucial that we constantly re-evaluate to find the optimal actuation. [3]

Here is the MPC algorithm. First, we set up everything required for the model predictive control loop. This consists of defining the duration of the trajectory, T, by choosing N and dt. Next, we define the vehicle model and constraints such as actual limitations. Finally, we define the cost function. [4]
<p align="center">
  <img src="https://github.com/jwangjie/SDC-MPC-Project/blob/master/Figures/MPC_setup.png" 
       width="680px" height="380px"/>
</p>

With the setup complete, we begin to state feedback loop. First, we pass the current state to the model predictive controller. Next, the optimization solver is called. The solver uses the initial state, the model constraints and cost function to return a vector of control inputs that minimize the cost function. The solver we'll use is called IPOPT. 

We only execute the very first set of control inputs ([δ_1,a_1], steering angle and acceleration) to the vehicle. This brings the vehicle to a new state and then we repeat the process.
![alt text](https://github.com/jwangjie/SDC-MPC-Project/blob/master/Figures/MPC_loop.png)

## Reflections
### Process model
The process model used in MPC pipeline is a kinematic model is different with the [Berkeley paper](http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf), although they are similar. In fact, it is possible to use different models in different parts of your pipeline, depending on what levels of accuracy you need in different places. The principles of the model we present can be applied to add parameters into the model to make models fit purpose. 
### Timestep Length and Elapsed Duration 
The time step dt and the elapsed duration T were tuned by trial and error. Due to the fact that the system has 100 millisecond latency, it’s meaningless to set dt < 0.1s. There was no big difference among 0.12s, 0.15s, and even 0.2s for a low speed simulation tracking. For high speed situations, dt = 0.12s was found to be the best one. A similar situation was found for the elapsed time T. We set dt = 0.12s and N=10, thus T=1.2s. 
### Cost function 
The cost function consists of components to minimize the cross-track error, the heading error, the velocity error, the steering and throttle effort, and the gap between sequential steering and throttle values. Weights of each component were tuned and signed based on the relative importance affecting the driving performance. The cost function can be written as J = w_cet * cet + w_epsi * epsi + w_vel * v + w_delta * delta + w_a * a + w_delta_diff * delta_diff + w_a_diff * a_diff. It’s found these weights affected the driving performance significantly than expected. The w_cet and w_epsi were set to be big values to avoid off-track driving. The w_delta, w_a, w_delta_diff, and w_a_diff were set relatively low since minimizing actuators use and smoothing the driving are not the first priority. It’s noticed the w_a played an important role in achieving high speed driving. When the value of w_a was large, the driving speed was limited due to the acceleration was slow. 
### Latency
The system latency was solved by predicting the future states (current + latency) using the process model before sending these states to the IPOPT solver. Thus, the simulating hardware latency was fixed in the MPC pipeline. 
### Summary
It’s relatively easy to realize a decent tracking driving performance for the low speed situations. In order to achieve a high speed driving, the weights of each component of the cost function need to be carefully tuned. 

## References
[1] Camacho, Eduardo F., and Carlos A. Bordons. Model predictive control in the process industry. Springer Science & Business Media, 2012.

[2] Wahlin, Brian T., and Albert J. Clemmens. "Automatic downstream water-level feedback control of branching canal networks: theory." Journal of irrigation and drainage engineering 132.3 (2006): 198-207.

[3] Udacity, Self-driving Car Nanodegree, term 2, Lecture 19 Section 1

[4] Udacity, Self-driving Car Nanodegree, term 2, Lecture 19 Section 6


---
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
