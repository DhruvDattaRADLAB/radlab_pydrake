This repository contains python code and data to build an empirically backed full dynamic model of RoboBall II platform. Eventually we'll integate this into a control stack.

## Repository outline

`roboball_controllers/` is a directory that will houses controller prototypes to test with the simulated model. This is only for prototyping, eventually I want to switch this to integrate with the existing ros2 control stack (here)[]

`roboball_plant/` contains useful setup scripts to create modular models of the robot
 - `create_ball_plant.py` defines two useful functions:  
    `add_RoboBall_plant()`loads in the robot urdf as a multibody plant in different useful configurations       
    `update_bedliner_properties()` to update the systems hydroelastic contact parameters onthe fly  

`roboball_controllers/` a directory to house future controller to add to experiments files

`utilities/` contains other non-robot specific features that are used in the paper.
 - `compare_models.py` contains a class that wraps the rigid body model from [1]
 - `world_features.py` contains a function to add a plate to the simulation for the robot to roll on

## Environment Setup
Then run the following on the command line. This will create a virtual environment in the env/ folder. You may choose to call it something else, but you will need to update the `.gitignore`

```sh
python3 -m venv env
```

activate the virtual environment

```sh
source env/bin/activate
```

Finally, install drake

```sh
pip install drake
```

you are now ready to run the examples in this repo!


## Figure Generation
The following commands will run the following experiments to generate plots from the paper [3]

Generates the friction model comparison graph

```sh
python3 ball_plant/joint_modifiers.py
```
generates the friction models when implemented on the robot in the steering stand

```sh
python3 run_compare_friction_models.py
```

generates the model comparisons in the final section of the paper
```sh
python3 run_compare_steer_models.py
```

## Data Storage

All of the data is 



## Other Experiment Files

### Jumping Dynamics
Sometimes we observed the ball jumping, so the `run_takeoff_dynamics.py` implements a variation of spongs swing up controller [2] in tandem with the ball plant constrined within the drive stand. Running the following command will generate the response using Meshcats 2D render mode. You can see and edit the controller in `roboball_controllers/swing_up_controllers.py`

```sh
python3 run_takeoff_dynamics.py
```

### Optimizing 
There are multiple different parameters to optimize for the model

**BE CAREFUL NOT TO ALLCOATE TOO MUCH MEMORY TO THE TESTING:** The task might be killed by your computer if it is using too much resources and you will have to restart the optimization.

#### Contact parameter Optimization
Use `optimizing_contact.py` if you would like to optimize the contact parameter for the ball (Dissipation and Elastic Modulus).

Contact Parameter Workflow:
1. Change the target damping value to your specific value you want to optimize too. The target damping value must be an array, as the optimizer can optimize for multiple values in one session.
```python
target_damping_values = [<insert values>]
```
2. You can also change the specific amonut of trials and cpu cores you want to use for the task inside the study.optimize function. 
```python
study.optimize(cost_function, n_trials=300, n_jobs=12, callbacks=[log_trial]) # n_jobs allocates the cpu cores to parallelizes the tasks, making the tests run much synchronously.
```
3. Run the script
4. After the script runs, it will give you a csv file with a log of all of the outputs as well as the final best parameters to use.
5. Run the parameters in the `run_ramp_test.py` to validate the contact parameter output on the console (it will tell you the damping ratio, ensure it is the intended ratio).


#### Stiction Friction Parameter Optmization
You can use `run_compare_friction_models.py` to optimize the stiction parameters for the system.

Friction Parameters Workflow:
1. Import your real world data csv into the `<insert folder> currently in ramp_simulation/real_data` folder.
2. Go into go into the `data_tools` folder and change the file path in `file_path.py` to the imported csv.
3. Use `trim_file.py` to preprocess your files and make consistent alignment for accurate data validation.
4. Run `run_compare_friction_models.py` and you can compare the model with the real data.
5. If you want to optimize your parameters, uncomment out the following code:
```python
    # study = optuna.create_study(direction="minimize")
    # study.optimize(objective, n_trials=500)

    # # Extract best params
    # best_params = study.best_params
    # majd_stiction_params = [
    #     best_params['f_w'], 
    #     best_params['f_c'], 
    #     best_params['sigma'], 
    #     best_params['w_c'], 
    #     best_params['n']
    # ]
```
This will optimize the values so you can pick the best parameters that fit your real life data.

### Ramp Testing
To observe and tune the dynamics of the ball, we can use ramp testing to analyze how the movements of the sim and reality differ. `run_ramp_test.py` runs a virtual ball down a ramp, generating data that mimics real-world conditions. 

Here is the general workflow:
1. Import your real world data csv into the `<insert folder> currently in ramp_simulation/real_data` folder.
2. Go into go into the `data_tools` folder and change the file path in `file_path.py` to the imported csv.
3. Use `trim_file.py` to preprocess your files and make consistent alignment for accurate data validation.
4. Go to the file `<insert folder> currently in run_simulation / visualize_data.py`. This will print out the initial_quaternion that the simulation will need to run.
5. Enter the `run_ramp_test.py` file and perform the following change:

```python
q0 = [<initial_qw>, <initial_qx>, <initial_qy>, <initial_qz>, 0, 0, 0.675, 0, 0] 
```

6. You can finally run the `run_ramp_test.py` through 
```bash
python3 run_ramp_test.py
```
and the test should run on the local host and save the data in `ramp_simulation/simulated_data.csv` ***(THIS IS SUBJECT TO CHANGE)***. 

7. Analyze these results through the `visualize_data.py` file.


## References
[1] Prevecek et al. "Empirically Compensated Setpoint Tracking for Spherical Robots With Pressurized Soft-Shells," in IEEE Robotics and Automation Letters, vol. 10, no. 3, pp. 2136-2143, March 2025

[2] Mark W. Spong "Energy Based Control of a Class of Underactuated Mechanical Systems," in IFAC Proceedings Volumes, vol. 29, Issue 1, 1996

[3] Oevermann et al. "A Modular Dynamic Model for A Soft Pendulum-Driven Spherical
Robots Using Drake," submitted to IEEE Robotics and Automation Letters