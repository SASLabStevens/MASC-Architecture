# Multi Level Adaptation for Automatic Landing with Engine Failure under Turbulence Uncertainties

This repository contains MASC Online navigation scheme, with support of Non linear guidance logic, a high fidelity simulation environment(X-Plane 11), and User Datagram Protocol (UDP) based interface. The system takes in landing zone coordinates and position coordinates where engine is malfunction. It later navigates the engine-out airplane to the approachable safty landing position in real-time.
A demonstration of the system in clear weather can be found here
## 👀 Demo Videos


<p align="center">
  <img src="files/MASC_Xplane_Simulation.gif" width = "400" height = "225" alt="Alt text" title="Optional title"/>
  <!-- <img src="files/icra20_1.gif" width = "320" height = "180"/> -->
</p>


This picture below is MASC Online Path Planning Architecture.
<p align='center'>
    <img src="/TuningWaypointFollowerForFixedWingUAVExample/graph/MASC Autopilot.png" alt="drawing" width="700"/>
</p>

### 📖 Useful material

> [[Paper](https://arxiv.org/abs/2209.04132?context=eess)] &emsp; [[SITL Simulation Video In Clear Weather](https://drive.google.com/file/d/1WmX2kXaYxDslZy5klvJAoZgwgKunmYe_/view?usp=sharing)] &emsp;  [[SITL Simulation Video In Turbulent Weather](https://drive.google.com/file/d/1-ySVJM_X1KhCySdqqr3tfN-P7GjJw5o3/view?usp=sharing)] &emsp; [[PPT slides](https://drive.google.com/file/d/1hBzhVMKTlmYcVpDXX8WOoF6kMFxH03yL/view?usp=share_link)]<br>

> [[Stevens Institute of Technology SAS Lab](https://saslabstevens.github.io/)] &emsp;



## Features

- Nonlinear Guidance Logic
- Precise trajectory tracking
- High fidelity simulation environment(X-Plane)
- User Datagram Protocol (UDP) based Publisher and Subscriber
- Decoupled longitudinal and lateral motion control
- Light weight online path planning framework(Low computation demanded)

## 🔧 Dependencies

- [MATALB R2021a](https://www.mathworks.com/products/new_products/previous_release_overview.html)
- [DSP System Toolbox](https://www.mathworks.com/products/dsp-system.html)
- [UAV Toolbox](https://www.mathworks.com/products/uav.html)
- [Aerospace Blockset](https://www.mathworks.com/products/aerospace-blockset.html)


## 🔧 Installation


#### Set up input data channel and output data channel in X-Plane 11

   <p align='center'>
    <img src="/TuningWaypointFollowerForFixedWingUAVExample/graph/IO_configure.png" alt="drawing" width="300"/>
   </p>

#### Configure IP address in MATLAB/Simulink

- The fixedWingPathFollowing model integrates the nonlinear guidane logic , UDP intterface
  with the high fidelity simulation environment. This model is to extract necessary information
  from the airplane status output bus signal and feed them into the waypoint follower to form 
  a control loop. The model assembles the control and environment inputs for the guidance model
  block.
 
   ```
   open_system('fixedWingPathFollowing');
   ```
-  Configure sender and publisher: Please follow instructions shown in the picture to set up 
   IP adddress for subscriber and publisher. Likewise  IP address configuration box in X-Plane.
   
   <p align='center'>
    <img src="/TuningWaypointFollowerForFixedWingUAVExample/graph/UDP Configuration.png" alt="drawing" width="400"/>
   </p>
   


    
## Running MASC Examples
   
1.  Open the program in which define the engine-out latitude,longitude and altitude. and then first 
    to put airplane to wherever the engine is broken. And then click the stop button at the up right 
    corner of the X-Plane window. 
    [Client matlab program](https://github.com/haotiangu/XPlaneConnect.git)
2.  Set up the engine out global position in malfunction position configuration block
3.  Set up the airport coordinates in simulink framework
4.  Click the run button to start the simulink model first. After 10 to 15 seconds, click the start button in the up right
   corner of the X-Plane window. Only by doing this can avoid the possible X-plane crash.
   ```
   sim("fixedWingPathFollowing");
   ``` 
6.  Run command in the matlab terminal to end this simulation.
 
   ```
   close_system("fixedWingPathFollowing");
   ``` 

## Step Response Test

- Step response for high fidelity autopilot
   ```
    open_system('stepResponse');
   ```

- Step response for low fidelity autopilot
   ```
    open_system('uavStepResponse');
   ```
## Cite *MASC*
If MASC framework is helpful, please help to ⭐ this repo or recommend it to your friends 😊 <br>

Thank you for citing [our *MASC* paper](./Multi_Level_Adaptive_Control.pdf) if you use any of this code: 
```
@article{gu2022multi,
  title={Multi-level Adaptation for Automatic Landing with Engine Failure under Turbulent Weather},
  author={Gu, Haotian and Jafarnejadsani, Hamidreza},
  journal={arXiv preprint arXiv:2209.04132},
  year={2022}
}
```

## 📧 Contact

If you encounter some issue when reproducing this project, please email `haotiangu1993@gmail.com` or `hgu8@stevens.edu`.


