# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program



---

## Implementation

The definition of MPC in this case is to solve the constrained finite-time optimal control problem:

![equation](https://latex.codecogs.com/gif.latex?%5Cbg_white%20%5Cbegin%7Balign%7D%20u%5E%7B*%7D_t%20%28x%28t%29%29%20%3A%3D%20%5Ctextup%7Bargmin%7D_%7Bu_%7Bt%7D%7D%20%5Csum_%7Bk%3D0%7D%5E%7BN-1%7D%20J%28x_%7Bt&plus;k%7D%2Cu_%7Bt&plus;k%7D%29%20%5Cnotag%20%5C%5C%20s.t.%20%5Cquad%20x_%7Bt%7D%20%3D%20x%28t%29%20%5Cnotag%20%5C%5C%20x_%7Bt&plus;k&plus;1%7D%20%3D%20f%28x_%7Bt&plus;k%7D%2Cu_%7Bt&plus;k%7D%29%20%5Cnotag%20%5C%5C%20x_%7Bmin%7D%20%5Cleq%20x_%7Bt&plus;k%7D%20%5Cleq%20x_%7Bmax%7D%20%5Cnotag%20%5C%5C%20u_%7Bmin%7D%20%5Cleq%20u_%7Bt&plus;k%7D%20%5Cleq%20u_%7Bmax%7D%20%5Cnotag%20%5Cend%7Balign%7D)

Cost Function

The cost function used is as follows:

![equation](https://latex.codecogs.com/gif.latex?%5Cbg_white%20%5Clarge%20%5Cbegin%7Balign%7D%20J%28x_%7Bt&plus;k%7D%2Cu_%7Bt&plus;k%7D%29%20%3D%20%28z_%7Bdes_%7Bt&plus;k%7D%7D-z_%7Bt&plus;k%7D%29%5E%7BT%7DQ%28z_%7Bdes_%7Bt&plus;k%7D%7D-z_%7Bt&plus;k%7D%29%20&plus;%20u_%7Bt&plus;k%7D%5E%7BT%7DRu_%7Bt&plus;k%7D%20&plus;%20%28u_%7Bt&plus;k&plus;1%7D-u_%7Bk%7D%29%5E%7BT%7DS%28u_%7Bt&plus;k&plus;1%7D-u_%7Bt&plus;k%7D%29%20%5Cnotag%20%5C%5C%20where%20%5Cquad%20Q%20%5Cin%5E%7Bn_%7Bmeas%7D%20%5Ctimes%20n_%7Bmeas%7D%7D%2C%20R%20%5Cin%5E%7Bn_%7Bcontrols%7D%20%5Ctimes%20n_%7Bcontrols%7D%7D%20%2C%20S%20%5Cin%5E%7Bn_%7Bcontrols%7D%20%5Ctimes%20n_%7Bcontrols%7D%7D%20%5Cnotag%20%5Cend%7Balign%7D)

Q, R and S are diagonal matrices with diagonal values weighted on importance for the objective value

Model

In order to use Model Predictive Control, a constraint of a system model is required to generate a predicted trajectory.  

The following equations represent the system dynamics assuming the kinematic bicycle model.

The generic non-linear discrete system model takes the form of:

![equation](https://latex.codecogs.com/gif.latex?%5Cbg_white%20%5Clarge%20x_%7Bk&plus;1%7D%20%3D%20f%28x_%7Bk%7D%2Cu_%7Bk%7D%29)

In this case, the state vector is:

![equation](https://latex.codecogs.com/gif.latex?%5Cbg_white%20%5Clarge%20x%20%3D%20%5Bp_%7Bx%7D%2C%20p_%7By%7D%2C%20%5Cpsi%2C%20v%2C%20cte%2C%20%5Cpsi_%7Be%7D%5D)

The control vector is:

![equation](https://latex.codecogs.com/gif.latex?%5Cbg_white%20%5Clarge%20u%20%3D%20%5B%5Cdelta%2C%20a%5D)

The bicycle model is defined as follows:

![equation](https://latex.codecogs.com/gif.latex?%5Cbg_white%20%5Cbegin%7Balign%7D%20p_%7Bx_%7Bk&plus;1%7D%7D%20%3D%20p_%7Bx_%7Bk%7D%7D%20&plus;%20v_%7Bk%7D%20*%20cos%28%5Cpsi_%7Bk%7D*%5CDelta%7Bt%7D%29%20%5Cnotag%20%5C%5C%20p_%7By_%7Bk&plus;1%7D%7D%20%3D%20p_%7By_%7Bk%7D%7D%20&plus;%20v_%7Bk%7D%20*%20sin%28%5Cpsi_%7Bk%7D*%5CDelta%7Bt%7D%29%20%5Cnotag%20%5C%5C%20%5Cpsi_%7Bk&plus;1%7D%20%3D%20%5Cpsi_%7Bk%7D%20-%20%5Cfrac%7Bv_%7Bk%7D%7D%7BL_%7Bf%7D%7D*%5Cdelta_%7Bk%7D*%5CDelta%7Bt%7D%20%5Cnotag%20%5C%5C%20v_%7Bk&plus;1%7D%20%3D%20v_%7Bk%7D%20&plus;%20a_%7Bk%7D*%5CDelta%7Bt%7D%20%5Cnotag%20%5C%5C%20cte_%7Bk&plus;1%7D%20%3D%20%28p_%7By_%7Bdes%7D%7D-p_%7By_%7Bk%7D%7D%29&plus;%20v_%7Bk%7D%20*%20sin%28%5Cpsi_%7Be_%7Bk%7D%7D*%5CDelta%7Bt%7D%29%20%5Cnotag%20%5C%5C%20%5Cpsi_%7Be_%7Bk&plus;1%7D%7D%20%3D%20%28%5Cpsi_%7Bdes%7D-%5Cpsi_%7Bk%7D%29%20-%20%5Cfrac%7Bv_%7Bk%7D%7D%7BL_%7Bf%7D%7D*%5Cdelta_%7Bk%7D*%5CDelta%7Bt%7D%20%5Cnotag%20%5Cend%7Balign%7D)


State/Input Constraints

The input constraints are set to the maximum steer angle of +/- 25 deg and +/- 1 for the throttle.

Timestep and Horizon Length

N was selected as 10 and dt as set as 0.1 In order to approximate the infinite time horizon sufficiently, a large enough time horizon was selected and verified through trial and error whether the horizon was a good enough approximation. In order to reduce the computational complexity without losing dynamics lost by discretization, a time step of 0.1 was selected.

Waypoints

The desired commands are found using a polyfit of the reference trajectory and looking up the values at the current state.

![equation](https://latex.codecogs.com/gif.latex?%5Cbg_white%20%5Cbegin%7Balign%7D%20p_%7By_%7Bdes%7D%7D%20%3D%20polyfit%28p_%7Bx_%7Bdes%7D%7D%2Cp_%7By_%7Bdes%7D%7D%29_%7Bx_%7Bk%7D%7D%20%5Cnotag%20%5C%5C%20%5Cpsi_%7Bdes%7D%20%3D%20atan2%28%5Cfrac%7Bdp_%7By_%7Bdes%7D%7D%7D%7Bdp_%7Bx_%7Bdes%7D%7D%7D%29%20%5Cnotag%20%5Cend%7Balign%7D)

The waypoints are preprocessed by transforming the points from the inertial frame to the vehicle body frame. Since the relative position error can be computed in the inertial frame, the position error can be transformed into the vehicle body frame. This is done as follows:

![equation](https://latex.codecogs.com/gif.latex?%5Cbg_white%20%5Clarge%20%5Cbegin%7Balign%7D%20%5Cbegin%7Bbmatrix%7D%20p_I_%7Bx_%7Be%7D%7D%20%5C%5C%20p_I_%7By_%7Be%7D%7D%20%5Cend%7Bbmatrix%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%20p_I_%7Bx_%7Bdes%7D%7D%20%5C%5C%20p_I_%7By_%7Bdes%7D%7D%20%5Cend%7Bbmatrix%7D%20-%20%5Cbegin%7Bbmatrix%7D%20p_I_%7Bx_%7Bvehicle%7D%7D%20%5C%5C%20p_I_%7By_%7Bvehicle%7D%7D%20%5Cend%7Bbmatrix%7D%20%5Cnotag%20%5C%5C%20%5Cbegin%7Bbmatrix%7D%20p_B_%7Bx_%7Be%7D%7D%20%5C%5C%20p_B_%7By_%7Be%7D%7D%20%5Cend%7Bbmatrix%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%20cos%28%5Cpsi%29%20%26%20sin%28%5Cpsi%29%20%5C%5C%20-sin%28%5Cpsi%29%20%26%20cos%28%5Cpsi%29%20%5Cend%7Bbmatrix%7D%20%5Cbegin%7Bbmatrix%7D%20p_I_%7Bx_%7Be%7D%7D%20%5C%5C%20p_I_%7By_%7Be%7D%7D%20%5Cnotag%20%5Cend%7Bbmatrix%7D%20%5Cend%7Balign%7D)

Latency

The latency is accounted for by applying an additional state update before sending the updated current state to the solver. The additional update step uses the time dependent action as part of the system equations and adds it to the current state using the current control input.

This is shown as follows:

![equation](https://latex.codecogs.com/gif.latex?%5Cbg_white%20%5Clarge%20%5Cbegin%7Balign%7D%20p_B_%7Bx_%7Bk%7D%7D%20%3D%20v_%7Bk%7D%20*%5CDelta%7Bt_%7Blatency%7D%7D%29%20%5Cnotag%20%5C%5C%20p_B_%7By_%7Bk%7D%7D%20%3D%200%20%5Cnotag%20%5C%5C%20%5Cpsi_%7Bk%7D%20%5Cmathrel%7B&plus;%7D%3D%20-%20%5Cfrac%7Bv_%7Bk%7D%7D%7BL_%7Bf%7D%7D*%5Cdelta_%7Bk%7D*%5CDelta%7Bt_%7Blatency%7D%7D%20%5Cnotag%20%5C%5C%20v_%7Bk%7D%20%5Cmathrel%7B&plus;%7D%3D%20a_%7Bk%7D*%5CDelta%7Bt_%7Blatency%7D%7D%20%5Cnotag%20%5C%5C%20cte_%7Bk%7D%20%5Cmathrel%7B&plus;%7D%3D%20v_%7Bk%7D%20*%20sin%28%5Cpsi_%7Be_%7Bk%7D%7D*%5CDelta%7Bt_%7Blatency%7D%7D%29%20%5Cnotag%20%5C%5C%20%5Cpsi_%7Be_%7Bk%7D%7D%20%5Cmathrel%7B&plus;%7D%3D%20-%20%5Cfrac%7Bv_%7Bk%7D%7D%7BL_%7Bf%7D%7D*%5Cdelta_%7Bk%7D*%5CDelta%7Bt_%7Blatency%7D%7D%20%5Cnotag%20%5Cend%7Balign%7D)

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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
