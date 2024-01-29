# FRIDAY: Real-time Learning DNN-based Stable LQR controller for Nonlinear Systems under Uncertain Disturbances
**Takahito Fujimori**

This code is provided as a part of "FRIDAY: Real-time Learning DNN-based Stable LQR controller for Nonlinear Systems under Uncertain Disturbances"

## Overview 
In this experiment, FRIDAY controls a mass system with unknown dynamics to track reference trajectory. The experimental setup is where FRIDAY only knows the nominal model and attempts to estimate the unknown residual dynamics to cancel out in real-time. We compare the performance to that of an adaptive Baseline controller. We also demonstrate the prediction performance of the DNN to provide a view of how fast and precisely FRIDAY learns the dynamics. Finally we provide an intuition about how SN-DNNs are suitable for FRIDAY’s real-time estimation, compared to another popular data-driven estimator GPs in the light of the training runtime and the mean estimation error.

## Main loop
Each controller has three main steps:
1. Decide control input `u` 
2. Input to unknown dynamics and get `dx`  
3. Update state `x = x + dx*dt`

These three steps are repeated in for-loop to achieve better perfomance. For example, LQR controller works as follows:
```
  u_lqr[i]= -np.dot(K,x_lqr[i-1,:]-r[i,:]) + ur #1. Decide control input

  dx_lqr  = car.true_dynamics(x_lqr[i-1, :], u_lqr[i]+dist , t[i] ) #2. Input to unknown dynamics and get dx

  x_lqr[i,:]=x_lqr[i-1,:]+ dx_lqr*dt #3. Update state 
```
![download](https://github.com/SpaceTAKA/FRIDAY_CarSimu/assets/68802350/368b681c-f973-4ac7-b03a-c50fb1ff5e4a)


Then, we plot tracking data to compare their performance using `plot_graph`. We also show the prediction performance to demonstrate how fast and presicely FRIDAY learns, plotting the observed residual dynamics data `Residual` and `prelist` which stores all the prediciton data.

![predict](https://user-images.githubusercontent.com/68802350/196947560-11d3e72f-bea7-495a-bd15-bd758a245ed1.png)

Next, we explain all functions, methods and parameters in this experiments.

## Function Definitions
In this section, we define basic functions such as RuLU, Affine layer etc. Each function has `feedforward` and `feedback` methods for calculating gradient through backpropagation.
| Function | Description |
----|---- 
| ReLU | Rectified Linear Unit layer |
| Affine | Affine layer with bias  |
| l2norm | calculating norm in L2-space  |
|IdentitywithLoss| Identity function in last layer with Sum of Squared Error|
|Optimizers| Stochastic Gradient Descent (SGD), Momentum SGD, Adagrad |
| FourLayerNet| a DNN composed of four fully-connected layers|

## Main class Definition
In this section, we define main class named 'PositioningCar' including the nominal model, the truth models and a graph plotter. To test FRIDAY perfomance in your own truth function, you change the contents of `true_dynamics` method.
| Method | Description |
----|---- 
| init       | the class obeject needs mass `m` and keyword `key` to select the unknown dynamics `*1`|
| true_dynamics | this method returns `dx` value through truth models  |
| model_matrix | this method returns `A,B` matrix of the nominal LTI system   |
| lqr | this method returns optimal feedback gain `K` of the nominal LTI (optional)   |
|plot_graph | this method plots the tracking trajectory `*2` |

`*1` `"nominal", "param", "multi", "enviro" ` are available.  
`*2` Using `plot_graph`, you can take a look at other states such as `vel`, `u`.


## Parameters and Variables
The parameters and variables of all contollers are as follows

**common parameters**
| Parameter | Description |
----|---- 
|m  | mass of  the vehicle|
| A, B| matrices of  LTI system|
| Q, R| weight for LQR (optional)|
| P, K | positive definite and feedback gain from LQR (optional)|
| T  | runnning time   |
| dt  | control loop period   |
| pr, vr| reference position and velocity|
| r  | reference state  |
| ur  | reference control input   |
| dist  | random disturbance noise on a control input   |
|Cycle | Wave period for tracking test (seconds)|

**FRIDAY**
| Parameter | Description |
----|---- 
| x_fri, u_fri| state and control input of FRIDAY|
| R_hat| the DNN approximation to residual dynamics |
| invB  |  alternative value of the inverse `B` due to  non-invertible `B`  `*1`   |
| LipR  |  intended value to constrain the Lipschitz constant of the DNN   |
| v1-v4, u1-u4  | singular vectors of the weight to calculate the singular value through power iteration method `*2`   |

`*1` Because the essence of FRIDAY is directly canceling residual dynamics in state-space, `invB` works just like inverse `B` in the mass model.  
`*2` Power iteration method is a computatinally inexpensive way to calculate singular value.


**LQR**
| Parameter | Description |
----|---- 
| x_lqr, u_lqr| state and control input of LQR|

**Baseline**
| Parameter | Description |
----|---- 
| x_base, u_base| state and control input of Baseline|
|  x_r | state of reference model|
| Ar, Br | matrices of reference model|
| u_n, u_a   | nominal input and adaptive input |
| sigma | basis function for representing uncertainty `*1`   |
| W_hat| time-variant weight of `sigma`  |
| gamma| learning rate  |

`*1` `sigma` consists of `(pos, vel, u)` just the same as FRIDAY.

**FRIDAY with GPs**
| Parameter | Description |
----|---- 
| x_gp, u_gp| state and control input of FRIDAY with GPs|
|  GPpre | the GP approximation to residual dynamics |
| kernel | the kernel of the GP model|




## Citation
The code here is for personal and educational use only; written permission from the author is required for further use. Please cite my work as follows.
