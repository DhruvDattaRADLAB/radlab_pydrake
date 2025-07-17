# RoboBall II creation files
This group of files contains helper functions when setting up Drake simulations with RoboBall. It mainly streamlines URDF load-in and provides a suite of modifiers to wire in to make the system more realistic.

```python
ballParams.py # a data class for holding filepaths to different robot urdfs
RoboBall_URDF/ # a ros2 package of the RoboBall URDF with validation documentation
data/ # a location for dynamics data used to verify friction, hydroelastic, and 3D roling dynamic model
```

## RoboBall Plant Creation (`create_ball_plant.py`)
The `create_ball_plant.py` contains three essential URDF loading functions. 
1) `add_RoboBall_plant()` loads in a different URDF model into a passed pydrake MultibodyPlant `plant` variable. The constraints are based on constructed drive and steer testing stands and a steer locking clamp to reduce the dynamics of the ball for a simple study. Please take a look at the function comments for specifics. 
2) `add_RoboBall_shell()` loads in a stripped-down URDF with outer shell without the pendulum. Micah used this heavily in [3-insert mutation paper]
3) `update_bedliner_properties()` This function will override the default URDF drake hydroelastic proximity properties in a simulation. this is useful for contct optimization studies

## Friction Models Description (`joint_modifiers.py`)

The `joint_modifiers.py` file contains two classes used to model various joint frictions. All of the functions use $v$ as the joint velocity. 

The first one a `StictionModel()` class is instantiated with a list of two parameters. The static friction coefficient ($\mu_s$), the dynamic friction coefficient ($\mu_d$), and the viscous damping coefficient ($c$). 

The following equation calculates the resulting resistive torque.

> $$ \tau_{friction} = \begin{cases}  \mu_s sign(v),  & \text{if } v < tol \\
                      -\mu_d sign(v) - cv & \text{otherwise} \end{cases}$$

The second is `StictionModel_majd()`, which follows a similar structure to the first. It is instantiated with a list of 5 parameters used in the equation.

The following equation calculates the resulting resistive torque [1]:

> $$\tau_{friction} = f_\omega v [f_c + \sigma e^{-|v/\omega_c|} - (\sigma+f_c) e^{-|n v /\omega_c|}]sign(v)$$


The parameters we found that made a good fit are summarized in the table below.

| -  | Stiction | Majd |
| ----------- | ----------- | ----------- |
| Parameter List | [ $\mu_s$, $\mu_d$, $c$] | [ $f_w$, $f_c$, $\sigma$, $\omega_c$, $n$]|
| Values | [0.7, 0.65, 0.104] |  [0.204, 0.45, 10, 0.1, 1] |

## FIRST Ball Dynamic Pressure Model (`pneumatics_system.py`)
The `pneumatics_system.py`  file contains a leaf system class that implements the pressure control system from RoboBall II using FIRST robotics components, as published in [2]. Adding the subsystem to a Drake simulation diagram will let you dynamically model changes in pressure. Based on the `manual` behavior, the system declares different required `InputPorts`; see the class file for more details on names. Running the `pneumatics_system.py` directly will show an example of the pressure system implementation.

*Note: as of Fall 2024 RoboBall II converted to ros2 instead of FIRST components so this model is outdated but still useful*

```sh
python3 roboball_plant/pnuematics_system.py
```

# References 

[1] V. J. Majd and M. A. Simaan, "A continuous friction model for servo systems with stiction," Proceedings of International Conference on Control Applications, Albany, NY, USA, 1995, pp. 296-301, doi: 10.1109/CCA.1995.555719.

[2] Oevermann et al. "A Pressure Model and Control System for a Pressurized Pendulum Driven Spherical Robot" in Proceedings ofthe 22nd International Conference on Ubiquitous Robots, College Station TX, July 2025

[3] Micahs Draft Nutation Paper
