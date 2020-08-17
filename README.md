# Model Predictive Control Project
Self-Driving Car Engineer Nanodegree Program

This project has 3 sub projects including:
1. [CarND-MPC](https://github.com/RuoyuLi92/CarND-MPC-Project/tree/master/CarND-MPC): Implementation of MPC to realize tracking of lake round track for automated-driving vehicle.
2. [CarND-MPC-Trajectory-Generation](https://github.com/RuoyuLi92/CarND-MPC-Project/tree/master/CarND-MPC-Trajectory-Generation): Implementation of a trajectory generator, the purpose is to generate circular trajectory in modified simulator and to record the trajectory for following model identification task. 
3. [CarND-MPC-Model-Identification](https://github.com/RuoyuLi92/CarND-MPC-Project/tree/master/CarND-MPC-Model-Identification): Implementation of a model identification application for MPC-Project.
	
## Environment set-up and instructions
The Docker relevant settings are the same for both 3 sub projects, it should be done only once. The controller prgramm is running in Docker Image, to run the Image with dependencies requried by this project, use the following command in Docker Quickstart Terminal:
_docker run -it --name <yourNameHere> -p 4567:4567 -v ‘pwd’:/work udacity/controls_kit:latest_

For security reasons, the VM does not automatically open port forwarding, so we need to manually enable port 4567. This is needed for the C++ program to successfully connect to the host simulator.

Port Forwarding Instructions
1. First open up Oracle VM VirtualBox
2. Click on the default session and select settings.
3. Click on Network, and then Advanced.
4. Click on Port Forwarding
5. Click on the green plus, adds new port forwarding rule.
6. Add a rule that assigns 4567 as both the host port and guest Port, as in the screenshot.

!["port-forward setting"](port-forward.png)

### MPC-Trajectory-Generation
This sub project involves a modified simulator which is included in repo [modified_sim](), run the following command to build and run the program:
 1. cd CarND-MPC-Trajectory-Generation
 2. mkdir build && cd build
 3. cmake .. && make
 4. mkdir output
 5. ./mpc

After seeing "Listening to port 4567", we can start "my_simulator" on host machine and choose MPC project to run, this time the track is no longer the round lake track in MPC-Project but a flat driving area
to generate circular trajectory using constant steering angle and throtle, the obtained trajectory variables will be stored in build/output/\*_vals.txt, where \* could be x, y, psi, v. The trajectory points will be used in
notebook[Estimate_Lf.ipynb](https://github.com/RuoyuLi92/CarND-MPC-Project/blob/master/CarND-MPC-Trajectory-Generation/jupyter/Estimate_Lf.ipynb) to fit the parametric model of trajectory, the center coordinate and radius of circular trajectory will be calculated. Results can be directly obtained from notebook [Estimate_Lf.ipyhb](https://github.com/RuoyuLi92/CarND-MPC-Project/blob/master/CarND-MPC-Trajectory-Generation/jupyter/Estimate_Lf.ipynb).

### MPC-Model-Identification
This sub project requires no simulator. This program is used to identify Vehicle parameter "Lf", i.e. distance between center of front wheel and vehicle center of mass. It is the only vehicle parameter used in MPC-Project. In
fact it is given in the course project, but I take this identification task as an additional exercise and the idea is to imitate a practical situation: we build a vehicle according to our design model, due to the manufacturing
and assembling error wear and etc., the designed parameter should be validated before we use it in the model predictive controller design. In our case, the vehicle model is existed in simulator, but we assume we dont know
exactly the vehicle parameter, but we can use vehicle tests in simulator to estimate the vehicle parameter.

The idea is to use x, y, psi, v, Lf as decision variables to formulate a optimization problem, the distance from generated trajectory points to the actual trajectory should be minimized. Kinematic model of the vehicle works
like constraints for the states, a special initialization using also the kinematic model is done for a better performance of the optimizer. Run the following command to build and run the program:

 1. cd CarND-Model-Identification
 2. mkdir build && cd build
 3. cmake .. && make
 4. ./mpc

Results will be stored in build/value_output.txt, the "value_output.txt" should be copied into the \CarND-MPC-Project/tree/master/CarND-MPC-Trajectory-Generation/build on host machine and the visualization will be done also in the notebook [Estimate_Lf.ipynb](https://github.com/RuoyuLi92/CarND-MPC-Project/blob/master/CarND-MPC-Trajectory-Generation/jupyter/Estimate_Lf.ipynb)

### MPC-Project
This project involves the Term 2 Simulator which could be downloaded [term2_sim](https://github.com/udacity/self-driving-car-sim/releases).

After installations and configurations are done, clone this repo into the docker Image, run the following command to build and run the program:
 1. cd CarND-MPC
 2. mkdir build && cd build
 3. cmake .. && make
 4. ./mpc

After seeing "Listening to port 4567", we can start the term2 simulator and choose MPC project to run.

## Simulating result
The MPC simulating result is demonstrated in the figure
!["result"](Udacity_cut_half.gif)
