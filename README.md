# CarND - Controls - Model Predictive control
Self-Driving Car Engineer Nanodegree Program

The goal of this project is to design a model predictive controller and test it in the Udacity simulator. The controller shall be able to handle system latency of 100 ms and drive the car safely around the track. 

As a requirement,  no tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). 

---
## The model

The model implemented in this project, is a kinematic model of the vehicle. The state of the system are vehicle position (x, y), orientation $\psi$, velocity v, cross-track error cte and orientation error $e\psi$.

The controller is fed with the current system state and a list of coordinates representing the reference trajectory and it calculates the values of steering angle $\delta$ and acceleration $a$ by solving a finite horizon optimization problem.

The optimization is carried out by IPOPT library, using the CPPAD library for the automatic handling of derivatives and hessians.

### Trajectory preprocessing

In order to use the CPPAD auto differentiation, the trajetory must be described as a function. In this project, we used a 3rd order polynomial fitted to the provided waypoints to fit the trajectory.

### Model equations

The vehicle follow a simple bycicle model :

$$
x_{t+1} = x_{t} + v_{t}*cos(\psi_{t})*dt
y_{t+1} = y_{t} + v_{t}*sin(\psi_{t})*dt
\psi_{t+1} = \psi_{t} + \frac{v_{t}}{L_{f}}*\delta_{t}*dt
v_{t+1} = v_{t} + a_{t}*dt
cte_{t+1} = f(x_{t}) - y_{t} + dt*v_{t}*sin(e\psi_{t})
e\psi_{t+1} = \psi_{t} - \psides_{t} + \frac{v_{t}}{L_{f}}*\delta_{t}*dt
$$

### Horizon and time step tuning

Since the model should be able to handle latency of 100 ms, it's advised to make the system predict at time intervals higher that the latency or the controller will lead to instability of the system due to overshooting of the trajectory. 

In order to make the system handle the high speeds, the dt was choosen as $ max(\frac{L_{f}}{v_{ref}}, latency) $.

The number of predicted steps was fixed to 10. This number was determined by simulations and was chosen as the highest possible number to make the car accelerates towards the desired speed without sacrificing computational performance.

### Latency handling

The commands computed by the controller will be delivered to the actuators in 100 ms on average. So, instead of feeding the controller with the telemetry data, we feed the model with our best estimate of the system's state at the time the command arrives in the actuators. In other words, we update the state using the kinematic equations described earlier with dt = 100 ms.

### Design of the objective function

The objective function is the key for the controller. We used the cost functional suggested in the lectures, but tuning the parameters according to trial runs. 

The key to a good performance is penalizing higher $e\psi$, discontinuities in the steeering angle and higher values of the steering angle. After the relative importance of those terms are set up, it's easy to tune other parameters and increase the reference speed. 


---
## Installation instructions

### Dependencies

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


### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

### Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

### Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

### Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

### Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

### Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

### Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

### How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
